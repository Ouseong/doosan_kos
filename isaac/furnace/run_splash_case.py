"""
판 투입 splash 1 건을 Newton (MPM 유체 + 강체 판, 양방향 결합) 으로 계산하고 재생용 npz 를 저장한다.

판 자세: pitch 0 = 로봇이 잡는 자세 그대로 세운 판 (짧은 변 W 가 수직, 긴 변 L 이 수평, 가장자리부터 진입),
        pitch 90 = 수평으로 눕힌 판. height = 판 최저점의 액면 위 높이 [m]. 판은 정지 상태에서 놓는다(free drop).

  ~/venvs/furnace_newton/bin/python isaac/furnace/run_splash_case.py --tag low_vertical --pitch 0 --height 0.06
  ~/venvs/furnace_newton/bin/python isaac/furnace/run_splash_case.py --tag high_flat --pitch 90 --height 0.5
출력: output/furnace/splash_cases/<tag>_trajectory.npz (재생용, 커밋 안 함), <tag>_timeseries.csv, <tag>.json
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "sim"))
sys.path.insert(0, HERE)

from fluid_backend import DropSpec, PlateFluidSim, quat_from_rpy, quat_rotate  # noqa: E402
from run_fluid_plate_test import load_cfg, write_rows  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
ap.add_argument("--tag", required=True)
ap.add_argument("--pitch", type=float, default=0.0, help="0=세운 판(가장자리 진입), 90=수평 판 [deg]")
ap.add_argument("--yaw", type=float, default=0.0)
ap.add_argument("--height", type=float, default=0.06, help="판 최저점의 액면 위 높이 [m]")
ap.add_argument("--vz", type=float, default=0.0, help="놓는 순간 하강 속도 [m/s] (아래 = 음수)")
ap.add_argument("--release_json", default=None, help="로봇 데모가 저장한 놓는 순간 판 상태 (있으면 pitch/height/vz 무시)")
ap.add_argument("--voxel", type=float, default=0.018)
ap.add_argument("--iters", type=int, default=150)
ap.add_argument("--duration", type=float, default=1.5)
ap.add_argument("--every", type=int, default=2, help="N 프레임마다 입자 스냅샷")
ap.add_argument("--sub", type=int, default=60000, help="재생용 입자 표본 수")
ap.add_argument("--out", default=os.path.join(ROOT, "output", "furnace", "splash_cases"))
args = ap.parse_args()

cfg = load_cfg(args.config, "low")
melt = cfg["furnace"]["melt_depth_m"]
lx, ly, lz = cfg["plate"]["size_m"]
q = quat_from_rpy([90.0 - args.pitch, 0.0, args.yaw])          # Newton 판 박스: x=L, y=W, z=t
corners = np.array([quat_rotate(q, [sx * lx / 2, sy * ly / 2, sz * lz / 2])
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
zc = melt + args.height - corners[:, 2].min()
drop = DropSpec(mode="free_drop", pos=(0.0, 0.0, float(zc)), quat=tuple(q), lin_vel=(0.0, 0.0, args.vz))
if args.release_json:
    rs = json.load(open(args.release_json))
    drop = DropSpec(mode="free_drop", pos=tuple(rs["pos"]), quat=tuple(rs["quat_xyzw"]),
                    lin_vel=tuple(rs["lin_vel"]), ang_vel=tuple(rs["ang_vel"]))
    print("[splash] release state from", args.release_json, "pos", np.round(rs["pos"], 3).tolist())

sim = PlateFluidSim(cfg, drop, voxel=args.voxel, max_iterations=args.iters)
res = sim.run(args.duration, snapshot_every=args.every, sub_count=args.sub)
os.makedirs(args.out, exist_ok=True)
rows = res["rows"]
write_rows(os.path.join(args.out, f"{args.tag}_timeseries.csv"), rows)
np.savez_compressed(os.path.join(args.out, f"{args.tag}_trajectory.npz"),
                    particles_sub=res["snapshots"], every_n_frames=args.every, fps=sim.fps,
                    plate=np.array([[r[k] for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")] for r in rows]),
                    plate_size=np.array(sim.plate_size), radius=sim.R, depth=sim.H, wall=sim.T, melt=sim.melt)
m = res["metrics"]
info = {"tag": args.tag, "release_json": args.release_json, "pitch_deg": args.pitch, "yaw_deg": args.yaw, "release_height_m": args.height,
        "release_vz": args.vz, "voxel_m": sim.voxel, "iterations": args.iters, "particles": res["particles"],
        "frames": res["frames"], "wall_s": round(res["wall_s"], 1), "plate_mass_kg": float(sim.model.body_mass.numpy()[sim.plate_body]),
        "metrics": m, "note": "Newton SolverImplicitMPM water-like isothermal fluid (not molten metal) + rigid plate, two-way coupled"}
json.dump(info, open(os.path.join(args.out, f"{args.tag}.json"), "w"), indent=2, default=float)
print(json.dumps({k: v for k, v in info.items() if k != "metrics"}, default=float))
print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()})
