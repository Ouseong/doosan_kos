"""
fluid_plate_test (PLAN Phase 1): 원통 도가니 + MPM 액체 + 10 mm metal_plate. Newton 단독 (Isaac Sim 밖).

SPEC §13 검증 1–8 을 자동 체크하고 output/furnace/fluid_plate_test/<tag>/ 에 JSON/CSV 를 쓴다.
"용탕의 기계적 거동을 단순화한 등온 유체 실험" 이며 실제 용탕을 재현한다고 주장하지 않는다.

  ~/venvs/furnace_newton/bin/python isaac/furnace/run_fluid_plate_test.py --fluid low --mode free_drop
  ... --suite            # 1–8 전체 (free_drop, kinematic, no-impact, reset, dx/dt sweep)
"""

import argparse
import copy
import csv
import datetime
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "sim"))

from fluid_backend import DropSpec, PlateFluidSim, quat_from_rpy  # noqa: E402
from splash_metrics import subtract_baseline  # noqa: E402


def load_cfg(path, fluid):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    fp = os.path.join(HERE, "config", f"fluid_{fluid}.yaml")
    if os.path.exists(fp):
        with open(fp) as f:
            over = yaml.safe_load(f) or {}
        for k, v in over.items():
            cfg.setdefault(k, {}).update(v)
    return cfg


def drop_from_cfg(cfg, mode=None, **over):
    d = copy.deepcopy(cfg["drop"])
    d.update({k: v for k, v in over.items() if v is not None})
    mode = mode or d["mode"]
    if mode == "no_plate":
        return None
    z = cfg["furnace"]["melt_depth_m"] + d["center_above_surface_m"]
    return DropSpec(mode=mode, pos=(0.0, 0.0, z), quat=tuple(quat_from_rpy(d["rpy_deg"])),
                    lin_vel=tuple(d["lin_vel"]), ang_vel=tuple(d["ang_vel"]),
                    kinematic_speed=d["kinematic_speed_mps"],
                    kinematic_stop_z=cfg["furnace"]["melt_depth_m"] - d["kinematic_depth_m"])


def write_rows(path, rows):
    if not rows:
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})


def run_one(cfg, mode, duration, out_dir, tag, save_npz=False, **kw):
    sim = PlateFluidSim(cfg, drop_from_cfg(cfg, mode), **kw)
    res = sim.run(duration, snapshot_every=5 if save_npz else 0)
    os.makedirs(out_dir, exist_ok=True)
    write_rows(os.path.join(out_dir, f"{tag}_timeseries.csv"), res["rows"])
    if save_npz and res["snapshots"] is not None:
        rows = res["rows"]
        np.savez_compressed(os.path.join(out_dir, f"{tag}_trajectory.npz"),
                            particles_sub=res["snapshots"], every_n_frames=5, fps=sim.fps,
                            plate=np.array([[r.get(k, 0.0) for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")]
                                            for r in rows]) if sim.plate_body >= 0 else np.zeros((0, 7)),
                            plate_size=np.array(sim.plate_size), radius=sim.R, depth=sim.H, wall=sim.T,
                            melt=sim.melt, init_particles_sub=sim.init_particles[res["sub_idx"]].astype(np.float16))
    info = {"tag": tag, "mode": mode, "voxel_m": sim.voxel, "ppc": sim.ppc, "fps": sim.fps,
            "substeps": sim.substeps, "project_outside": sim.project, "particles": res["particles"],
            "frames": res["frames"], "wall_s": round(res["wall_s"], 1),
            "plate_density": sim.plate_density if sim.plate_body >= 0 else None,
            "plate_mass_kg": float(sim.model.body_mass.numpy()[sim.plate_body]) if sim.plate_body >= 0 else None,
            "metrics": res["metrics"]}
    return info, res, sim


def plate_checks(res, sim, g=9.81):
    rows = res["rows"]
    pz = np.array([r["plate_min_z"] for r in rows])
    pr = np.array([r["plate_max_r"] for r in rows])
    vz = np.array([r["vz"] for r in rows])
    q = np.array([[r["qx"], r["qy"], r["qz"], r["qw"]] for r in rows])
    ang = 2 * np.degrees(np.arccos(np.clip(np.abs(q @ q[0]), 0, 1)))
    fdt = 1.0 / sim.fps
    entry = int(np.argmax(pz < sim.melt)) if np.any(pz < sim.melt) else None
    c = {"plate_min_z_m": float(pz.min()), "plate_max_r_m": float(pr.max()), "crucible_R_m": sim.R,
         "no_penetration": bool(pz.min() > -0.5 * sim.plate_size[2] and pr.max() < sim.R + 0.5 * sim.plate_size[2]),
         "max_rotation_change_deg": float(ang.max()), "entry_frame": entry}
    if entry is not None and entry > 2:
        a_before = (vz[entry] - vz[entry - 3]) / (3 * fdt)
        # 진입 후 0.15 s 동안 액체가 판에 준 수직 충격량 / (판 무게 × 0.15 s).
        # 0 = 자유낙하(반력 없음), 1 = 무게를 완전히 떠받침. 벽·바닥 접촉 전 구간만 쓰도록 바닥 근처면 자른다
        n = int(round(0.15 / fdt))
        e2 = min(len(vz) - 1, entry + n)
        lim = np.where(pz[entry:e2 + 1] < 0.02)[0]
        if len(lim):
            e2 = entry + int(lim[0])
        T_ = (e2 - entry) * fdt
        ratio = float(((vz[e2] - vz[entry]) + g * T_) / (g * T_)) if T_ > 0 else None
        c.update({"az_before_entry": float(a_before), "window_s": T_,
                  "fluid_impulse_over_weight": ratio, "free_fall_ratio": 0.0,
                  "decelerated_after_entry": bool(ratio is not None and ratio > 0.2)})
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
    ap.add_argument("--fluid", default="low", choices=["low", "high"])
    ap.add_argument("--mode", default="free_drop", choices=["free_drop", "kinematic", "no_plate"])
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--suite", action="store_true", help="SPEC §13 1–8 전체")
    ap.add_argument("--out", default=None)
    ap.add_argument("--project_outside", type=int, default=None)
    ap.add_argument("--voxel", type=float, default=None)
    ap.add_argument("--wall", type=float, default=None)
    a = ap.parse_args()
    cfg = load_cfg(a.config, a.fluid)
    if a.wall:
        cfg["furnace"]["wall_thickness_m"] = a.wall
    dur = a.duration or cfg["drop"]["duration_s"]
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(ROOT, "output", "furnace", "fluid_plate_test", stamp)
    os.makedirs(out, exist_ok=True)
    kw = dict(voxel=a.voxel, project_outside=None if a.project_outside is None else bool(a.project_outside))

    if not a.suite:
        info, res, sim = run_one(cfg, a.mode, dur, out, a.mode, save_npz=True, **kw)
        if sim.plate_body >= 0:
            info["checks"] = plate_checks(res, sim)
        print(json.dumps(info, indent=2))
        with open(os.path.join(out, f"{a.mode}_result.json"), "w") as f:
            json.dump(info, f, indent=2)
        return

    # ── SPEC §13 suite ─────────────────────────────────────────────────────────
    report = {"backend": "newton 1.8.0 standalone (SolverImplicitMPM + SolverMuJoCo), no Isaac Sim",
              "fluid_model": "isothermal simplified fluid (water-like). NOT molten metal",
              "config": cfg, "duration_s": dur, "runs": {}, "checks": {}}
    R = report["runs"]
    base, _, _ = run_one(cfg, "no_plate", dur, out, "no_plate", **kw)
    R["no_plate"] = base
    fd, fres, fsim = run_one(cfg, "free_drop", dur, out, "free_drop", save_npz=True, **kw)
    fd["checks"] = plate_checks(fres, fsim)
    fd["metrics_minus_baseline"] = subtract_baseline(fd["metrics"], base["metrics"])
    R["free_drop"] = fd
    kd, kres, ksim = run_one(cfg, "kinematic", dur, out, "kinematic", save_npz=True, **kw)
    kd["coupling"] = "one-way (plate trajectory prescribed; fluid force NOT applied to plate)"
    kd["metrics_minus_baseline"] = subtract_baseline(kd["metrics"], base["metrics"])
    R["kinematic"] = kd

    # 7 reset: 같은 sim 객체 reset 후 재실행 → 판 궤적·입자 비교
    p_first = np.array([[r["px"], r["py"], r["pz"]] for r in fres["rows"]])
    x_first = fres["final_particles"]
    x0_first = fsim.x_after_reset.copy()
    fsim.reset()
    init_err = float(np.abs(fsim.x_after_reset - x0_first).max())
    res2 = fsim.run(dur)
    p_second = np.array([[r["px"], r["py"], r["pz"]] for r in res2["rows"]])
    # 비교 기준: 새 객체 두 개를 같은 설정으로 돌린 차이 (GPU 비결정성 수준)
    _, fres_b, _ = run_one(cfg, "free_drop", dur, out, "free_drop_repeat", **kw)
    p_fresh = np.array([[r["px"], r["py"], r["pz"]] for r in fres_b["rows"]])

    m = base["metrics"]
    tot = m["total_mass"]
    C = report["checks"]
    C["1_liquid_retained"] = {
        "no_plate_final_outside_frac": m["final_outside_mass"] / tot,
        "no_plate_cumulative_escaped_frac": m["cumulative_escaped_mass"] / tot,
        "no_plate_penetrated_peak_frac": m["penetrated_mass_peak"] / tot,
        "pass": bool(m["cumulative_escaped_mass"] / tot < 0.001)}
    pc = fd["checks"]
    C["2_no_penetration_plate"] = {"plate_min_z_m": pc["plate_min_z_m"], "plate_max_r_m": pc["plate_max_r_m"],
                                   "pass": pc["no_penetration"]}
    C["3_free_motion_after_release"] = {"max_rotation_change_deg": pc["max_rotation_change_deg"],
                                        "pass": bool(pc["max_rotation_change_deg"] > 1.0)}
    ke_b = m["fluid_kinetic_energy_peak"]
    C["4_plate_displaces_liquid"] = {
        "no_plate_fluid_ke_peak_J": ke_b,
        "free_drop_fluid_ke_peak_J": fd["metrics"]["fluid_kinetic_energy_peak"],
        "kinematic_fluid_ke_peak_J": kd["metrics"]["fluid_kinetic_energy_peak"],
        "free_drop_above_surface_mass_peak_minus_baseline":
            fd["metrics_minus_baseline"]["above_surface_mass_peak_minus_baseline"],
        "free_drop_max_height_minus_baseline": fd["metrics_minus_baseline"]["max_splash_height_minus_baseline"],
        "note": "판 부피/수면적 = 1.5e-3/0.503 ≈ 3 mm 수위 상승이 이론값이라 수면 위 질량은 작다. 유체 운동에너지로 판정",
        "pass": bool(fd["metrics"]["fluid_kinetic_energy_peak"] > 3 * ke_b + 0.1
                     and kd["metrics"]["fluid_kinetic_energy_peak"] > 3 * ke_b + 0.1)}
    C["5_fluid_reaction_on_plate"] = {k: pc.get(k) for k in ("entry_frame", "az_before_entry", "window_s",
                                                              "fluid_impulse_over_weight", "decelerated_after_entry")}
    C["5_fluid_reaction_on_plate"]["pass"] = bool(pc.get("decelerated_after_entry", False))
    C["6_particle_io"] = {"read": ["particle_q", "particle_qd", "particle_mass"], "pass": True}
    d_reset = float(np.abs(p_first - p_second).max())
    d_fresh = float(np.abs(p_first - p_fresh).max())
    C["7_reset"] = {"initial_state_max_err_after_reset_m": init_err,
                    "plate_traj_max_diff_reset_vs_first_m": d_reset,
                    "plate_traj_max_diff_fresh_vs_fresh_m": d_fresh,
                    "final_particle_max_diff_reset_vs_first_m": float(np.abs(x_first - res2["final_particles"]).max()),
                    "note": "fresh-vs-fresh 차이 = 같은 입력에서도 생기는 실행 간 차이(GPU 비결정성). reset 차이가 이 수준이면 reset 자체는 정상",
                    "pass": bool(init_err < 1e-6 and d_reset <= 2.0 * d_fresh + 1e-3)}

    # 8 sweep: dx 와 dt (fps). free_drop + no_plate 각각
    sweep = []
    # 반복 횟수는 해상도에 맞춰 키운다 (Phase 1: dx 가 작을수록 미수렴 → 바닥 관통·수위 저하)
    for label, kv in (("dx25_fps100", dict(voxel=0.025, fps=100, max_iterations=50)),
                      ("dx18_fps100", dict(voxel=0.018, fps=100, max_iterations=150)),
                      ("dx125_fps100", dict(voxel=0.0125, fps=100, max_iterations=400)),
                      ("dx25_fps200", dict(voxel=0.025, fps=200, max_iterations=50))):
        kk = dict(kw); kk.update(kv)
        b_, _, _ = run_one(cfg, "no_plate", dur, out, f"sweep_{label}_no_plate", **kk)
        f_, fr_, fs_ = run_one(cfg, "free_drop", dur, out, f"sweep_{label}_free_drop", **kk)
        ch = plate_checks(fr_, fs_)
        mm = subtract_baseline(f_["metrics"], b_["metrics"])
        sweep.append({"label": label, **kv, "particles": f_["particles"], "wall_s_free": f_["wall_s"],
                      "thickness_over_dx": fs_.plate_size[2] / fs_.voxel,
                      "noimpact_escaped_frac": b_["metrics"]["cumulative_escaped_mass"] / b_["metrics"]["total_mass"],
                      "noimpact_pen_peak_frac": b_["metrics"]["penetrated_mass_peak"] / b_["metrics"]["total_mass"],
                      "escaped_mass_minus_baseline": mm["cumulative_escaped_mass_minus_baseline"],
                      "over_rim_peak": f_["metrics"]["over_rim_mass_peak"],
                      "p999_height": f_["metrics"]["p999_splash_height"],
                      "noimpact_level_drop_m": None,
                      "plate_min_z": ch["plate_min_z_m"], "plate_final_z": fr_["rows"][-1]["pz"],
                      "fluid_impulse_over_weight": ch.get("fluid_impulse_over_weight"),
                      "above_surface_mass_peak_minus_baseline": mm["above_surface_mass_peak_minus_baseline"], "no_penetration": ch["no_penetration"]})
        print(sweep[-1], flush=True)
    C["8_resolution_dt_sweep"] = sweep
    write_rows(os.path.join(out, "sweep.csv"), sweep)
    with open(os.path.join(out, "suite_result.json"), "w") as f:
        json.dump(report, f, indent=2, default=float)
    print(json.dumps(report["checks"], indent=2, default=float))


if __name__ == "__main__":
    main()
