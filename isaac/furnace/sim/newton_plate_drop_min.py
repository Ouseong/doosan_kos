"""
Phase 0 최소 예제: 용기 + 액체(MPM) + 떨어지는 10 mm 강체판 (Newton 단독, Isaac Sim 없음).

구조는 설치본 newton 1.8.0 의 examples/mpm/example_mpm_twoway_coupling.py 를 그대로 따른다
(강체 = SolverMuJoCo, 유체 = SolverImplicitMPM, 양방향 결합 = 예제의 impulse 교환 커널 재사용).
물 재질 값(friction 0, tensile_yield_ratio 1, viscosity 0)은 examples/mpm/example_mpm_water_dam_break.py 기본값.

- "용탕의 기계적 거동을 단순화한 등온 유체" 실험이다. 실제 용탕 아님.
- 판 이름은 metal_plate. 재질 미확정 → 밀도는 후보 중 하나를 --density 로 고른다.
- 용기 벽은 정적 박스 5개(바닥 + 4면). 판·액체 모두 이 벽과 충돌해야 한다.
  dam_break 예제의 _project_inside_tank 같은 "입자를 강제로 용기 안에 되돌리는" 보정은 쓰지 않는다
  (누수가 있으면 그대로 결과에 나오게).

실행 (호스트 venv):
  ~/venvs/furnace_newton/bin/python isaac/furnace/sim/newton_plate_drop_min.py --out output/furnace/phase0/newton_min
"""

import argparse
import json
import os
import time

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM
from newton.examples.mpm.example_mpm_twoway_coupling import compute_body_forces, subtract_body_force

DENSITY_CANDIDATES = [2700.0, 7140.0, 7870.0]   # PLAN §7 plate.density_candidates (재질 미확정, 예시)

p = argparse.ArgumentParser()
p.add_argument("--out", default="output/furnace/phase0/newton_min")
p.add_argument("--density", type=float, default=DENSITY_CANDIDATES[0])
p.add_argument("--voxel", type=float, default=0.025, help="MPM 격자 크기 [m]")
p.add_argument("--ppc", type=float, default=3.0, help="축당 셀당 입자 수")
p.add_argument("--fps", type=float, default=100.0)
p.add_argument("--substeps", type=int, default=4)
p.add_argument("--duration", type=float, default=1.5)
p.add_argument("--tank_half", type=float, nargs=2, default=[0.30, 0.30])
p.add_argument("--tank_height", type=float, default=0.70)
p.add_argument("--wall", type=float, default=0.05)
p.add_argument("--water_depth", type=float, default=0.40)
p.add_argument("--plate_size", type=float, nargs=3, default=[0.50, 0.30, 0.010])
p.add_argument("--drop_clearance", type=float, default=0.25, help="액면 위 판 중심 높이 [m]")
p.add_argument("--pitch_deg", type=float, default=20.0, help="판 초기 기울기 (y 축 회전)")
p.add_argument("--no_plate", action="store_true", help="no-impact 대조 (판 없음)")
args = p.parse_args()
os.makedirs(args.out, exist_ok=True)

hx_t, hy_t = args.tank_half
H, T = args.tank_height, args.wall
lx, ly, lz = args.plate_size

# ── 강체 모델: 용기 벽(정적) + 판(자유 강체) ───────────────────────────────────
builder = newton.ModelBuilder()
builder.default_shape_cfg.mu = 0.5
wall_cfg = newton.ModelBuilder.ShapeConfig(density=0.0, mu=0.5)
builder.add_ground_plane(cfg=wall_cfg)                       # 용기 바닥 = z 0
for sx, sy, ex, ey in ((1, 0, T / 2, hy_t + T), (-1, 0, T / 2, hy_t + T),
                       (0, 1, hx_t + T, T / 2), (0, -1, hx_t + T, T / 2)):
    cx = sx * (hx_t + T / 2)
    cy = sy * (hy_t + T / 2)
    builder.add_shape_box(-1, xform=wp.transform(p=wp.vec3(cx, cy, H / 2), q=wp.quat_identity()),
                          hx=ex, hy=ey, hz=H / 2, cfg=wall_cfg, label="tank_wall")

plate_body = -1
surface_z = args.water_depth
if not args.no_plate:
    q0 = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(np.deg2rad(args.pitch_deg)))
    plate_body = builder.add_body(
        xform=wp.transform(p=wp.vec3(0.0, 0.0, surface_z + args.drop_clearance), q=q0),
        label="metal_plate")
    builder.add_shape_box(plate_body, hx=lx / 2, hy=ly / 2, hz=lz / 2,
                          cfg=newton.ModelBuilder.ShapeConfig(density=args.density, mu=0.5),
                          label="metal_plate")

# ── 유체 모델: 용기 안 물 블록 ──────────────────────────────────────────────────
fluid_builder = newton.ModelBuilder()
SolverImplicitMPM.register_custom_attributes(fluid_builder)
lo = np.array([-hx_t, -hy_t, 0.0])
hi = np.array([hx_t, hy_t, args.water_depth])
res = np.ceil(args.ppc * (hi - lo) / args.voxel).astype(int)
cell = (hi - lo) / res
radius = 0.5 * float(np.max(cell))
fluid_density = 1000.0
fluid_builder.add_particle_grid(
    pos=wp.vec3(lo + 0.5 * cell), rot=wp.quat_identity(), vel=wp.vec3(0.0),
    dim_x=int(res[0]), dim_y=int(res[1]), dim_z=int(res[2]),
    cell_x=float(cell[0]), cell_y=float(cell[1]), cell_z=float(cell[2]),
    mass=float(np.prod(cell) * fluid_density), jitter=2.0 * radius, radius_mean=radius,
    flags=newton.ParticleFlags.ACTIVE)

model = builder.finalize()
fluid_model = fluid_builder.finalize()
fluid_model.mpm.friction.fill_(0.0)
fluid_model.mpm.tensile_yield_ratio.fill_(1.0)
fluid_model.mpm.viscosity.fill_(0.0)

cfg = SolverImplicitMPM.Config()
cfg.voxel_size = args.voxel
cfg.max_iterations = 50
cfg.strain_basis = "P0"
mpm = SolverImplicitMPM(fluid_model, config=cfg)
mpm.setup_collider(model=model)          # 벽·바닥·판 모두 MPM 충돌체 (twoway 예제와 같은 방식)

# 판이 없으면(no-impact) 강체가 없어 MuJoCo 모델을 만들 수 없다 → 강체 단계 생략
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, njmax=200) if plate_body >= 0 else None
s0, s1 = model.state(), model.state()
fs = fluid_model.state()
if s0.body_q is not None:
    fs.body_q = wp.empty_like(s0.body_q)
    fs.body_qd = wp.empty_like(s0.body_qd)
    fs.body_f = wp.empty_like(s0.body_f)
fs.mpm.particle_Jp.fill_(1.0)
control = model.control()
pipeline = newton.CollisionPipeline(model)
contacts = pipeline.contacts()
if solver is not None:
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)

frame_dt = 1.0 / args.fps
sim_dt = frame_dt / args.substeps
max_nodes = 1 << 20
imp = wp.zeros(max_nodes, dtype=wp.vec3, device=model.device)
imp_pos = wp.zeros(max_nodes, dtype=wp.vec3, device=model.device)
imp_ids = wp.full(max_nodes, value=-1, dtype=int, device=model.device)
body_fluid_f = wp.zeros_like(s0.body_f) if s0.body_f is not None else None


def collect():
    a, b, c = mpm.collect_collider_impulses(fs)
    imp_ids.fill_(-1)
    n = min(a.shape[0], max_nodes)
    imp[:n].assign(a[:n]); imp_pos[:n].assign(b[:n]); imp_ids[:n].assign(c[:n])


collect()
mpm.project_outside(fs, fs, sim_dt)

n_frames = int(round(args.duration * args.fps))
mass = fluid_model.particle_mass.numpy()
total_mass = float(mass.sum())
log = {k: [] for k in ("t", "plate_pos", "plate_quat", "plate_vel", "inside_mass", "outside_mass",
                       "below_floor_mass", "over_rim_mass", "max_particle_z", "p99_particle_z")}
snap_frames = sorted(set([0, n_frames // 4, n_frames // 2, n_frames - 1]))
snaps = {}
rng = np.random.default_rng(0)
sub_idx = rng.choice(fluid_model.particle_count, size=min(60000, fluid_model.particle_count), replace=False)
traj_sub = []

t0 = time.time()
for f in range(n_frames):
    for _ in range(args.substeps if solver is not None else 0):
        s0.clear_forces()
        wp.launch(compute_body_forces, dim=imp_ids.shape[0],
                  inputs=[frame_dt, imp_ids, imp, imp_pos, mpm.collider_body_index,
                          s0.body_q, model.body_com, s0.body_f])
        body_fluid_f.assign(s0.body_f)
        pipeline.collide(s0, contacts)
        solver.step(s0, s1, control, contacts, sim_dt)
        s0, s1 = s1, s0
    if solver is not None and fs.body_q is not None and fs.body_q.shape[0] > 0:
        wp.launch(subtract_body_force, dim=fs.body_q.shape,
                  inputs=[frame_dt, s0.body_q, s0.body_qd, body_fluid_f, model.body_inv_inertia,
                          model.body_inv_mass, fs.body_q, fs.body_qd])
    mpm.step(fs, fs, contacts=None, control=None, dt=frame_dt)
    collect()

    x = fs.particle_q.numpy()
    inside = (np.abs(x[:, 0]) <= hx_t) & (np.abs(x[:, 1]) <= hy_t) & (x[:, 2] >= -0.5 * args.voxel)
    log["t"].append((f + 1) * frame_dt)
    log["inside_mass"].append(float(mass[inside].sum()))
    log["outside_mass"].append(float(mass[~inside].sum()))
    log["below_floor_mass"].append(float(mass[x[:, 2] < -0.5 * args.voxel].sum()))
    log["over_rim_mass"].append(float(mass[x[:, 2] > H].sum()))
    log["max_particle_z"].append(float(x[:, 2].max()))
    log["p99_particle_z"].append(float(np.percentile(x[:, 2], 99)))
    if plate_body >= 0:
        bq = s0.body_q.numpy()[plate_body]
        bqd = s0.body_qd.numpy()[plate_body]
        log["plate_pos"].append(bq[:3].tolist())
        log["plate_quat"].append(bq[3:].tolist())      # warp transform: p(3) + q(xyzw)
        log["plate_vel"].append(bqd[:3].tolist())
    if f in snap_frames:
        snaps[f] = x.copy()
    if f % 5 == 0:
        traj_sub.append(x[sub_idx].astype(np.float16))
wall_s = time.time() - t0

# ── 자동 체크 (Phase 0 최소: SPEC §13 의 1,2,3,4,5,6 을 대략 확인) ────────────────
res_json = {
    "backend": "newton (standalone venv, no Isaac Sim)",
    "newton": newton.__version__, "warp": wp.__version__,
    "coupling": "two-way (MPM collider impulses -> rigid body forces, as in example_mpm_twoway_coupling)",
    "rigid_solver": "SolverMuJoCo(use_mujoco_contacts=False)",
    "fluid": {"model": "implicit MPM, water-like (friction 0, viscosity 0)", "density": fluid_density,
              "isothermal_simplified": True, "particles": int(fluid_model.particle_count),
              "voxel_m": args.voxel, "ppc": args.ppc, "total_mass_kg": total_mass},
    "tank": {"inner_half_m": [hx_t, hy_t], "height_m": H, "walls": "5 static boxes (convex pieces)"},
    "plate": None if args.no_plate else {"name": "metal_plate", "size_m": args.plate_size,
                                         "density_candidate": args.density,
                                         "mass_kg": float(model.body_mass.numpy()[plate_body]),
                                         "thickness_over_voxel": lz / args.voxel},
    "fps": args.fps, "substeps": args.substeps, "frames": n_frames, "wall_time_s": round(wall_s, 1),
    "checks": {},
}
c = res_json["checks"]
c["1_liquid_retained"] = {"final_outside_mass_kg": log["outside_mass"][-1],
                          "final_outside_fraction": log["outside_mass"][-1] / total_mass,
                          "below_floor_mass_kg_max": max(log["below_floor_mass"])}
c["6_particle_read"] = {"ok": True, "fields": ["particle_q", "particle_qd", "particle_mass"]}
if plate_body >= 0:
    P = np.array(log["plate_pos"]); V = np.array(log["plate_vel"]); Q = np.array(log["plate_quat"])
    # 판 꼭짓점 최저 z (관통 확인)
    corners = np.array([[sx * lx / 2, sy * ly / 2, sz * lz / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])

    def rot(q, v):   # q = xyzw
        x_, y_, z_, w_ = q
        u = np.array([x_, y_, z_])
        return v + 2 * np.cross(u, np.cross(u, v) + w_ * v)

    zmin = np.array([min(rot(Q[i], cc)[2] for cc in corners) + P[i, 2] for i in range(len(P))])
    xyext = np.array([max(np.abs(rot(Q[i], cc)[:2] + P[i, :2]).max() for cc in corners) for i in range(len(P))])
    c["2_no_penetration"] = {"plate_min_z_m": float(zmin.min()), "floor_z_m": 0.0,
                             "plate_max_abs_xy_m": float(xyext.max()),
                             "tank_inner_half_m": [hx_t, hy_t],
                             "ok": bool(zmin.min() > -0.005 and xyext.max() < max(hx_t, hy_t) + 0.005)}
    # 3: 자유 운동 (회전 변화)
    ang = [2 * np.degrees(np.arccos(min(1.0, abs(float(np.dot(Q[0], Q[i])))))) for i in range(len(Q))]
    c["3_free_motion"] = {"max_rotation_change_deg": float(max(ang)),
                          "plate_z_travel_m": float(P[0, 2] - P[:, 2].min())}
    # 5: 액체 반력 → 진입 후 감속. 진입 = 판 최저점이 초기 액면 아래로 내려간 첫 프레임
    entry = int(np.argmax(zmin < surface_z)) if np.any(zmin < surface_z) else None
    g = 9.81
    if entry is not None and entry + 5 < len(V):
        vz_entry = V[entry, 2]
        vz_after = V[entry + 5, 2]
        dt5 = 5 * frame_dt
        a_obs = (vz_after - vz_entry) / dt5
        c["5_fluid_reaction"] = {"entry_frame": entry, "vz_at_entry": float(vz_entry),
                                 "vz_5_frames_later": float(vz_after),
                                 "observed_az_m_s2": float(a_obs), "free_fall_az_m_s2": -g,
                                 "decelerated_vs_free_fall": bool(a_obs > -g * 0.5)}
    else:
        c["5_fluid_reaction"] = {"entry_frame": entry, "note": "plate did not reach liquid in time"}
    # 4: 판이 액체를 밀어냄 (최고 입자 높이 상승)
    c["4_liquid_displaced"] = {"initial_max_z": float(snaps[0][:, 2].max()) if 0 in snaps else None,
                               "peak_max_particle_z": float(max(log["max_particle_z"])),
                               "peak_p99_particle_z": float(max(log["p99_particle_z"])),
                               "over_rim_mass_peak_kg": float(max(log["over_rim_mass"]))}

with open(os.path.join(args.out, "result.json"), "w") as f:
    json.dump(res_json, f, indent=2)
with open(os.path.join(args.out, "timeseries.csv"), "w") as f:
    keys = ["t", "inside_mass", "outside_mass", "below_floor_mass", "over_rim_mass",
            "max_particle_z", "p99_particle_z"]
    hdr = keys + (["plate_x", "plate_y", "plate_z", "plate_qx", "plate_qy", "plate_qz", "plate_qw",
                   "plate_vx", "plate_vy", "plate_vz"] if plate_body >= 0 else [])
    f.write(",".join(hdr) + "\n")
    for i in range(n_frames):
        row = [log[k][i] for k in keys]
        if plate_body >= 0:
            row += log["plate_pos"][i] + log["plate_quat"][i] + log["plate_vel"][i]
        f.write(",".join(f"{v:.6g}" for v in row) + "\n")
np.savez_compressed(os.path.join(args.out, "trajectory.npz"),        # gitignore 대상 (재생용)
                    particles_sub=np.array(traj_sub), sub_idx=sub_idx, every_n_frames=5,
                    plate_pos=np.array(log["plate_pos"]), plate_quat=np.array(log["plate_quat"]),
                    plate_size=np.array(args.plate_size), tank_half=np.array([hx_t, hy_t]), tank_height=H,
                    **{f"snap_{k}": v for k, v in snaps.items()})
print(json.dumps(res_json, indent=2))
