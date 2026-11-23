"""
Newton (단독) 유체-판 백엔드. 원통 도가니 + MPM 액체 + metal_plate.

- 설치본 newton 1.8.0 의 examples/mpm/example_mpm_twoway_coupling.py 구조를 따른다.
  양방향 결합 커널(compute_body_forces, subtract_body_force)은 그 예제에서 import 해서 그대로 쓴다.
- 용기 벽: 원통 링을 convex 박스 N 조각 + 바닥 박스로 구성 (단일 convex hull 아님).
- 모드:
    free_drop  판 = 자유 강체 (SolverMuJoCo). 액체 ↔ 판 양방향 (two-way)
    kinematic  판 = 정해진 궤적으로 움직이는 kinematic body. 액체는 판에 밀리지만 판은 반력을 안 받음 (one-way)
    no_plate   판 없음 (no-impact 대조 → 수치 누수 기준선)
- 입자 위치/속도/질량은 매 프레임 numpy 로 읽어 splash_metrics.SplashAccumulator 에 넣는다.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM
from newton.examples.mpm.example_mpm_twoway_coupling import compute_body_forces, subtract_body_force

from splash_metrics import CylinderVessel, SplashAccumulator


def quat_from_rpy(rpy_deg):
    r, p, y = np.deg2rad(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2), np.cos(y / 2), np.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([x, yy, z, w])          # warp 순서 xyzw


def quat_rotate(q, v):
    u, w = np.asarray(q[:3]), q[3]
    v = np.asarray(v, dtype=float)
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


@dataclass
class DropSpec:
    mode: str = "free_drop"
    pos: tuple = (0.0, 0.0, 0.75)            # 판 중심 초기 위치 (world)
    quat: tuple = (0.0, 0.0, 0.0, 1.0)       # xyzw
    lin_vel: tuple = (0.0, 0.0, 0.0)
    ang_vel: tuple = (0.0, 0.0, 0.0)
    kinematic_speed: float = 1.0
    kinematic_stop_z: float = 0.25


class PlateFluidSim:
    def __init__(self, cfg: dict, drop: DropSpec | None, *, voxel=None, ppc=None, fps=None, substeps=None,
                 density=None, fluid_density=None, project_outside=None, seed=0, device=None,
                 max_iterations=None):
        self.cfg = cfg
        fr = cfg["fluid_res"]
        self.voxel = voxel or fr["voxel_m"]
        self.ppc = ppc or fr["particles_per_cell"]
        self.fps = fps or fr["fps"]
        self.substeps = substeps or fr["substeps"]
        self.project = fr["project_outside"] if project_outside is None else project_outside
        self.settle_s = fr.get("settle_s", 0.0)
        self.max_iterations = max_iterations or fr["max_iterations"]
        f = cfg["furnace"]
        self.R = f["crucible_inner_d_m"] / 2
        self.H = f["crucible_depth_m"]
        self.T = f["wall_thickness_m"]
        self.melt = f["melt_depth_m"]
        pl = cfg["plate"]
        self.plate_size = pl["size_m"]
        self.plate_density = density if density is not None else pl["density_candidates"][pl["density_index"]]
        self.fluid_density = fluid_density or cfg["fluid"]["density"]
        self.drop = drop
        self.seed = seed
        self.device = device
        self._build()

    # ── 모델 구성 ─────────────────────────────────────────────────────────────
    def _build(self):
        R, H, T = self.R, self.H, self.T
        b = newton.ModelBuilder()
        b.default_shape_cfg.mu = 0.5
        wcfg = newton.ModelBuilder.ShapeConfig(density=0.0, mu=0.5)
        n = self.cfg["furnace"]["wall_segments"]
        half_tan = np.pi * (R + T) / n * 1.08               # 조각끼리 살짝 겹치게
        for k in range(n):
            a = 2 * np.pi * k / n
            c = (R + T / 2) * np.array([np.cos(a), np.sin(a)])
            q = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), float(a))
            b.add_shape_box(-1, xform=wp.transform(p=wp.vec3(float(c[0]), float(c[1]), H / 2), q=q),
                            hx=T / 2, hy=float(half_tan), hz=H / 2 + T / 2, cfg=wcfg, label=f"crucible_wall_{k}")
        b.add_shape_box(-1, xform=wp.transform(p=wp.vec3(0.0, 0.0, -T / 2), q=wp.quat_identity()),
                        hx=R + T, hy=R + T, hz=T / 2, cfg=wcfg, label="crucible_floor")

        self.plate_body = -1
        d = self.drop
        if d is not None and d.mode in ("free_drop", "kinematic"):
            lx, ly, lz = self.plate_size
            self.plate_body = b.add_body(
                xform=wp.transform(p=wp.vec3(*map(float, d.pos)), q=wp.quat(*map(float, d.quat))),
                label="metal_plate", is_kinematic=(d.mode == "kinematic"))
            b.add_shape_box(self.plate_body, hx=lx / 2, hy=ly / 2, hz=lz / 2,
                            cfg=newton.ModelBuilder.ShapeConfig(density=float(self.plate_density),
                                                                mu=float(self.cfg["plate"]["friction"])),
                            label="metal_plate")

        fb = newton.ModelBuilder()
        SolverImplicitMPM.register_custom_attributes(fb)
        sp = self.voxel / self.ppc
        rng = np.random.default_rng(self.seed)
        g = np.arange(-R + sp / 2, R, sp)
        gz = np.arange(sp / 2, self.melt, sp)
        X, Y, Z = np.meshgrid(g, g, gz, indexing="ij")
        P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        P += rng.uniform(-0.5, 0.5, P.shape) * sp * 0.9      # 결정적 jitter (seed 고정)
        P = P[(np.hypot(P[:, 0], P[:, 1]) < R - 0.25 * sp) & (P[:, 2] > 0.0) & (P[:, 2] < self.melt)]
        m = float(sp ** 3 * self.fluid_density)
        fb.add_particles(pos=P.tolist(), vel=np.zeros_like(P).tolist(), mass=[m] * len(P),
                         radius=[0.5 * sp] * len(P))
        self.init_particles = P

        self.model = b.finalize(device=self.device)
        self.fmodel = fb.finalize(device=self.device)
        fl = self.cfg["fluid"]
        self.fmodel.mpm.friction.fill_(float(fl["friction"]))
        self.fmodel.mpm.tensile_yield_ratio.fill_(float(fl["tensile_yield_ratio"]))
        self.fmodel.mpm.viscosity.fill_(float(fl["viscosity"]))

        self.reset()

    def _make_solvers(self):
        """solver 는 내부 상태(충돌체 직전 위치, warm start 등)를 가진다. reset 때마다 새로 만든다.
        (같은 solver 를 재사용하면 reset 후 첫 step 에서 판이 원위치로 '순간이동' 한 것으로 보여
        결과가 달라지는 것을 Phase 1 에서 확인했다.)"""
        mc = SolverImplicitMPM.Config()
        mc.voxel_size = self.voxel
        fr = self.cfg["fluid_res"]
        mc.max_iterations = self.max_iterations
        mc.tolerance = fr.get("tolerance", mc.tolerance)
        mc.strain_basis = fr.get("strain_basis", "P0")
        for k in ("warmstart_mode", "collider_basis", "velocity_basis", "solver", "critical_fraction", "integration_scheme"):
            if k in fr:
                setattr(mc, k, fr[k])
        self.mpm = SolverImplicitMPM(self.fmodel, config=mc)
        self.mpm.setup_collider(model=self.model)
        self.rigid = None
        if self.plate_body >= 0 and self.drop.mode == "free_drop":
            self.rigid = newton.solvers.SolverMuJoCo(self.model, use_mujoco_contacts=False, njmax=400)
            self.pipeline = newton.CollisionPipeline(self.model)
            self.contacts = self.pipeline.contacts()
            self.control = self.model.control()

    # ── reset: 같은 초기 상태로 되돌린다 (SPEC §13-7) ─────────────────────────
    def reset(self):
        self.s0, self.s1 = self.model.state(), self.model.state()
        self.plate_mode = None if self.plate_body < 0 else self.drop.mode
        self.fs = self.fmodel.state()
        if self.s0.body_q is not None:
            self.fs.body_q = wp.empty_like(self.s0.body_q)
            self.fs.body_qd = wp.empty_like(self.s0.body_qd)
            self.fs.body_f = wp.empty_like(self.s0.body_f)
        self.fs.mpm.particle_Jp.fill_(1.0)
        if self.plate_body >= 0:
            d = self.drop
            qd = np.zeros((self.model.body_count, 6), dtype=np.float32)
            qd[self.plate_body, :3] = d.lin_vel
            qd[self.plate_body, 3:] = d.ang_vel
            self.s0.body_qd.assign(qd)
            if d.mode == "free_drop":
                # 자유 관절 속도도 맞춘다 (MuJoCo 는 joint_qd 를 쓴다)
                jqd = self.model.joint_qd.numpy()
                jqd[:6] = np.concatenate([d.lin_vel, d.ang_vel])
                self.model.joint_qd.assign(jqd)
                newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.s0)
        self._make_solvers()
        self.t = 0.0
        self.frame_dt = 1.0 / self.fps
        nmax = 1 << 21
        dev = self.model.device
        self.imp = wp.zeros(nmax, dtype=wp.vec3, device=dev)
        self.imp_pos = wp.zeros(nmax, dtype=wp.vec3, device=dev)
        self.imp_ids = wp.full(nmax, value=-1, dtype=int, device=dev)
        self.body_fluid_f = wp.zeros_like(self.s0.body_f) if self.s0.body_f is not None else None
        self._collect()
        self.mpm.project_outside(self.fs, self.fs, self.frame_dt)
        self.x_after_reset = self.fs.particle_q.numpy()      # 결정적 초기 상태 (settle 전)
        # settle: 판을 공중에 둔 채(정지, 접촉 없음) 액체만 먼저 가라앉힌다. 격자 배치 초기 과도응답을
        # 판 진입 결과와 섞지 않기 위해서다. 이 구간은 지표에 넣지 않고 t=0 은 settle 이 끝난 시점.
        if self.fs.body_q is not None:
            self.fs.body_q.assign(self.s0.body_q)
            self.fs.body_qd.zero_()
        for _ in range(int(round(self.settle_s * self.fps))):
            self.mpm.step(self.fs, self.fs, contacts=None, control=None, dt=self.frame_dt)
            if self.project:
                self.mpm.project_outside(self.fs, self.fs, self.frame_dt)
        self._collect()
        if self.fs.body_q is not None:
            self.fs.body_qd.assign(self.s0.body_qd)

    def _collect(self):
        a, b_, c = self.mpm.collect_collider_impulses(self.fs)
        self.imp_ids.fill_(-1)
        n = min(a.shape[0], self.imp_ids.shape[0])
        self.imp[:n].assign(a[:n]); self.imp_pos[:n].assign(b_[:n]); self.imp_ids[:n].assign(c[:n])

    def _kinematic_pose(self, t):
        d = self.drop
        z = max(d.kinematic_stop_z, d.pos[2] - d.kinematic_speed * t)
        vz = -d.kinematic_speed if z > d.kinematic_stop_z else 0.0
        return np.array([d.pos[0], d.pos[1], z, *d.quat], dtype=np.float32), np.array([0, 0, vz, 0, 0, 0], np.float32)

    # ── 한 프레임 ─────────────────────────────────────────────────────────────
    def step(self):
        fdt = self.frame_dt
        if self.rigid is not None:
            sdt = fdt / self.substeps
            for _ in range(self.substeps):
                self.s0.clear_forces()
                wp.launch(compute_body_forces, dim=self.imp_ids.shape[0],
                          inputs=[fdt, self.imp_ids, self.imp, self.imp_pos, self.mpm.collider_body_index,
                                  self.s0.body_q, self.model.body_com, self.s0.body_f])
                self.body_fluid_f.assign(self.s0.body_f)
                self.pipeline.collide(self.s0, self.contacts)
                self.rigid.step(self.s0, self.s1, self.control, self.contacts, sdt)
                self.s0, self.s1 = self.s1, self.s0
            wp.launch(subtract_body_force, dim=self.fs.body_q.shape,
                      inputs=[fdt, self.s0.body_q, self.s0.body_qd, self.body_fluid_f,
                              self.model.body_inv_inertia, self.model.body_inv_mass,
                              self.fs.body_q, self.fs.body_qd])
        elif self.plate_body >= 0:                     # kinematic: 궤적을 직접 쓴다 (one-way)
            q, qd = self._kinematic_pose(self.t + fdt)
            bq = self.s0.body_q.numpy(); bq[self.plate_body] = q
            bqd = self.s0.body_qd.numpy(); bqd[self.plate_body] = qd
            self.s0.body_q.assign(bq); self.s0.body_qd.assign(bqd)
            self.fs.body_q.assign(bq); self.fs.body_qd.assign(bqd)
        self.mpm.step(self.fs, self.fs, contacts=None, control=None, dt=fdt)
        if self.project:
            self.mpm.project_outside(self.fs, self.fs, fdt)
        self._collect()
        self.t += fdt

    def plate_state(self):
        if self.plate_body < 0:
            return None
        bq = self.s0.body_q.numpy()[self.plate_body]
        bqd = self.s0.body_qd.numpy()[self.plate_body]
        return bq[:3].copy(), bq[3:].copy(), bqd[:3].copy(), bqd[3:].copy()

    def plate_corners_world(self):
        st = self.plate_state()
        lx, ly, lz = self.plate_size
        cs = [[sx * lx / 2, sy * ly / 2, sz * lz / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        return np.array([quat_rotate(st[1], c) + st[0] for c in cs])

    # ── 실행 ─────────────────────────────────────────────────────────────────
    def run(self, duration, snapshot_every=0, sub_count=50000):
        vessel = CylinderVessel(radius=self.R, rim_z=self.H, wall=self.T, tol=0.5 * self.voxel)
        mass = self.fmodel.particle_mass.numpy()
        acc = SplashAccumulator(vessel, mass, surface_z0=self.melt)
        n = int(round(duration * self.fps))
        rows, snaps = [], []
        rng = np.random.default_rng(0)
        sub = rng.choice(len(mass), size=min(sub_count, len(mass)), replace=False)
        t0 = time.time()
        for f in range(n):
            self.step()
            x = self.fs.particle_q.numpy()
            v = self.fs.particle_qd.numpy()
            acc.update(self.t, x, v)
            row = {"t": self.t}
            if self.plate_body >= 0:
                p, q, lv, av = self.plate_state()
                cz = self.plate_corners_world()
                row.update(dict(zip(["px", "py", "pz"], p)), **dict(zip(["qx", "qy", "qz", "qw"], q)),
                           **dict(zip(["vx", "vy", "vz"], lv)), **dict(zip(["wx", "wy", "wz"], av)),
                           plate_min_z=float(cz[:, 2].min()),
                           plate_max_r=float(np.hypot(cz[:, 0], cz[:, 1]).max()))
            for k in ("outside_mass", "penetrated_mass", "over_rim_mass", "max_z", "p999_z"):
                row[k] = acc.series[k][-1]
            rows.append(row)
            if snapshot_every and f % snapshot_every == 0:
                snaps.append(x[sub].astype(np.float16))
        wall = time.time() - t0
        return {"metrics": acc.summary(), "rows": rows, "wall_s": wall, "frames": n,
                "particles": int(len(mass)), "snapshots": np.array(snaps) if snaps else None, "sub_idx": sub,
                "final_particles": self.fs.particle_q.numpy()}
