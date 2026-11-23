"""
evaluate (PLAN Phase 6): baseline 과 추정기 조건 비교. 유체는 Newton 단독 (dx 25 mm 저해상도로 탐색, 상위 후보만 고해상도 재검증).

같은 seed → 같은 실제 파지 오차. 조건:
  A 고정 투입 / B 핀 변위 추정 보정 / C oracle(GT) / D 낮은 높이 조심 투입
실험 묶음:
  main    : 같은 release 높이에서 A/B/C 비교 + D  (추정 효과)
  height  : 조건 A 로 release 높이·기울기 sweep    (낙하 높이 효과, 추정 효과와 분리)
  repeat  : 같은 입력 반복 → 지표의 실행 간 잡음
  highres : 상위 후보 몇 개를 dx 12.5 mm 로 재검증
결과: output/furnace/evaluate/<run>/results.csv, summary.json

  ~/venvs/furnace_newton/bin/python isaac/furnace/run_evaluate.py --conditions A,B,C,D --seeds 0-19
"""

import argparse
import copy
import csv
import datetime
import json
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "sim"))

import grasp_geometry as G  # noqa: E402
from fluid_backend import DropSpec, PlateFluidSim  # noqa: E402
from grasp_estimator import as_err, estimate  # noqa: E402
from m1013_kinematics import M1013, make_T  # noqa: E402
from release_controller import command, pose_error, quat_xyzw, released_plate, target_plate_pose  # noqa: E402
from splash_metrics import subtract_baseline  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--config", default=os.path.join(HERE, "config", "default.yaml"))
ap.add_argument("--conditions", default="A,B,C,D")
ap.add_argument("--seeds", default="0-19")
ap.add_argument("--groups", default="main,height,repeat,highres")
ap.add_argument("--height", type=float, default=0.15, help="main: 판 하단의 액면 위 높이 [m]")
ap.add_argument("--low_height", type=float, default=0.06, help="조건 D 높이 [m]. dx 25 mm 의 2 칸 이상 (0.03 m 는 settle 중 판 충돌체가 액면과 겹쳐 비물리적 비산)")
ap.add_argument("--pitch", type=float, default=0.0, help="main: 목표 판 기울기 [deg] (0=수직, 90=수평)")
ap.add_argument("--duration", type=float, default=1.0)
ap.add_argument("--out", default=None)
a = ap.parse_args()

cfg = yaml.safe_load(open(a.config))
lay = cfg["layout"]
fx, fy = lay["furnace_xy"]
rim = cfg["furnace"]["rim_height_from_floor_m"]
floor_z = rim - cfg["furnace"]["crucible_depth_m"]
melt_w = floor_z + cfg["furnace"]["melt_depth_m"]
PS = tuple(cfg["plate"]["size_m"])
INS = cfg["grasp"]["insert_depth_m"]
R_cr = cfg["furnace"]["crucible_inner_d_m"] / 2
kin = M1013()
base_T = make_T(np.array([0, 0, lay["robot_base_z"]]))
s0, s1 = (int(v) for v in a.seeds.split("-")) if "-" in a.seeds else (int(a.seeds), int(a.seeds))
seeds = list(range(s0, s1 + 1))
conds = a.conditions.split(",")
groups = a.groups.split(",")
out = a.out or os.path.join(ROOT, "output", "furnace", "evaluate", datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
os.makedirs(out, exist_ok=True)
WALL_MARGIN = 0.02           # 목표 release 자세에서 판 모서리 ~ 도가니 내벽 최소 거리
CENTER_OFFSET = 0.25          # 판 중심 = 도가니 중심에서 로봇 쪽 0.25 m (Phase 4 도달성 하한. 0.30 이면 모서리가 벽(반경 0.39 m)에 닿음)

ERR_RANGE = dict(dx=0.002, dy=0.020, dz=0.008, rx=np.deg2rad(3), ry=np.deg2rad(3), rz=np.deg2rad(3))   # [가정]
PIN_NOISE = 1e-4              # 핀 변위 측정 잡음 [가정]
P_TO_N = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)   # 열 = N.x, N.y, N.z 를 P 좌표로 (N.x=P.y, N.y=P.z, N.z=P.x)


def sample_err(seed):
    r = np.random.default_rng(1000 + seed)
    return {k: float(r.uniform(-v, v)) for k, v in ERR_RANGE.items()}


def observe(err_true, seed):
    r = np.random.default_rng(2000 + seed)
    dp, dn, _, _ = G.pin_displacements(G.plate_in_S(PS, INS, err_true), PS, 0.0)
    dp = np.where(dp > 0, dp + r.normal(0, PIN_NOISE, 25), 0)
    dn = np.where(dn > 0, dn + r.normal(0, PIN_NOISE, 25), 0)
    s = G.shift_from_opening(0.0)
    return estimate(dp, dn, s, s, INS)            # GT 입력 없음


def to_newton(T_W):
    T = T_W.copy()
    T[:3, 3] -= np.array([fx, fy, floor_z])
    return T


baseline_cache = {}


def baseline(voxel, iters):
    key = (voxel, iters)
    if key not in baseline_cache:
        sim = PlateFluidSim(cfg, None, voxel=voxel, max_iterations=iters)
        baseline_cache[key] = sim.run(a.duration)["metrics"]
    return baseline_cache[key]


def run_trial(group, cond, seed, height, pitch, roll=0.0, voxel=0.025, iters=50, vz=0.0):
    err_true = sample_err(seed)
    est = observe(err_true, seed)
    err_used = {"A": {}, "D": {}, "B": as_err(est), "C": err_true}[cond]
    h = a.low_height if cond == "D" else height
    # 목표: 판 최저점이 액면 위 h. 기울기에 따라 판 중심 높이를 맞춘다
    T0 = target_plate_pose(np.zeros(3), pitch, roll)
    corners = np.array([[sx * PS[2] / 2, sy * PS[0] / 2, sz * PS[1] / 2] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    zmin_rel = min((T0[:3, :3] @ c)[2] for c in corners)
    # 판 중심 오프셋: 기본 CENTER_OFFSET(도달성 하한). 기울인 판은 x 로 더 퍼지므로 모든 모서리가 도가니 벽에서
    # WALL_MARGIN 안쪽에 들도록 중심을 도가니 중심 쪽으로 당긴다 (도달 못 하면 robot_reachable=False 로 표시)
    off = CENTER_OFFSET
    for _ in range(60):
        cr = max(np.hypot(*(T0[:3, :3] @ c + np.array([-off, 0, 0]))[:2]) for c in corners)
        if cr <= R_cr - WALL_MARGIN:
            break
        off -= 0.005
    T_target = target_plate_pose(np.array([fx - off, fy, melt_w + h - zmin_rel]), pitch, roll)
    T_S = command(T_target, PS, INS, err_used)
    T_rel = released_plate(T_S, PS, INS, err_true)
    pe, ae = pose_error(T_rel, T_target)
    q, ik_pe, ik_re = kin.ik(G.link6_from_gripper_S(T_S), base=base_T, seeds=8)
    reachable = bool(ik_pe < 0.002 and ik_re < np.deg2rad(1))
    Tn = to_newton(T_rel)
    # 축 규약 변환: grasp_geometry 판 프레임 P (x=두께, y=긴 변, z=짧은 변) → Newton 판 박스 N (x=긴 변, y=짧은 변, z=두께)
    R_N = Tn[:3, :3] @ P_TO_N
    drop = DropSpec(mode="free_drop", pos=tuple(Tn[:3, 3]), quat=tuple(quat_xyzw(R_N)),
                    lin_vel=(0.0, 0.0, vz), ang_vel=(0.0, 0.0, 0.0))
    t0 = time.time()
    sim = PlateFluidSim(cfg, drop, voxel=voxel, max_iterations=iters, seed=0)
    res = sim.run(a.duration)
    rows = res["rows"]
    m = subtract_baseline(res["metrics"], baseline(voxel, iters))
    max_r = max(r["plate_max_r"] for r in rows)
    final = rows[-1]
    # release 순간 판이 도가니 벽과 겹치는가 (Newton 좌표: 벽 = 반경 R~R+T, 높이 0~depth+T/2)
    T_n = Tn                                     # 판 프레임 P 기준 (corners 도 P 기준)
    cw = np.array([T_n[:3, :3] @ c + T_n[:3, 3] for c in corners])
    r_c = np.hypot(cw[:, 0], cw[:, 1])
    initial_overlap = bool(np.any((r_c > R_cr - 0.003) & (cw[:, 2] < cfg["furnace"]["crucible_depth_m"] + cfg["furnace"]["wall_thickness_m"] / 2)))
    # 진입 중 벽 접촉: release 부터 판 최저점이 액면 아래 0.10 m 에 닿을 때까지만 본다 (가라앉은 뒤 표류 제외)
    surf = cfg["furnace"]["melt_depth_m"]
    entry = []
    for r in rows:
        entry.append(r)
        if r["plate_min_z"] < surf - 0.10:
            break
    entry_max_r = max(r["plate_max_r"] for r in entry)
    # 그리퍼 하단(릴리스 위치에 그대로 있다고 가정)에 유체가 닿았는가: 그리퍼 S 원점 아래 0.16 m 높이 (Newton 좌표)
    grip_bottom_z = (to_newton(T_S) @ np.array([0, G.PIN_Y[2], G.PIN_Z[0] - 0.01, 1]))[2]
    gripper_wet = bool(max(r["max_z"] for r in rows) >= grip_bottom_z)
    row = {"group": group, "condition": cond, "seed": seed, "release_height_m": h, "pitch_deg": pitch, "roll_deg": roll,
           "voxel_m": voxel, "release_vz": vz,
           **{f"err_true_{k}": v for k, v in err_true.items()},
           **{f"err_used_{k}": err_used.get(k, 0.0) for k in ERR_RANGE},
           "release_pos_err_mm": pe * 1000, "release_ang_err_deg": ae, "robot_reachable": reachable,
           "target_center_offset_m": off, "initial_overlap_invalid": initial_overlap,
           "entry_wall_contact": bool(entry_max_r > R_cr - 0.003), "entry_plate_max_r_m": entry_max_r,
           "plate_inserted": bool(not initial_overlap and entry_max_r <= R_cr - 0.003
                                  and final["plate_min_z"] < cfg["furnace"]["melt_depth_m"]),
           "wall_contact_any_time": bool(max_r > R_cr - 0.003), "plate_max_r_m": max_r,
           "collision_gripper_fluid": gripper_wet, "task_time_s": None,
           **{k: v for k, v in m.items() if k != "total_mass"}, "wall_s": round(time.time() - t0, 1)}
    return row


results = []
csv_path = os.path.join(out, "results.csv")


def flush():
    keys = sorted({k for r in results for k in r}, key=lambda k: (k not in ("group", "condition", "seed"), k))
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in results:
            w.writerow(r)


def log(r):
    results.append(r)
    flush()
    print(f"[eval] {r['group']:7s} {r['condition']} s{r['seed']:02d} h={r['release_height_m']:.2f} p={r['pitch_deg']:.0f} "
          f"off={r['target_center_offset_m']:.3f} ovl={r['initial_overlap_invalid']} reach={r['robot_reachable']} "
          f"pos_err={r['release_pos_err_mm']:.1f}mm ang_err={r['release_ang_err_deg']:.2f}° entry_wall={r['entry_wall_contact']} "
          f"esc={r['cumulative_escaped_mass_minus_baseline']:.3f} rim={r['over_rim_mass_peak']:.3f} "
          f"h999={r['p999_splash_height']:.3f} ke={r['fluid_kinetic_energy_peak']:.2f} ({r['wall_s']}s)", flush=True)


if "main" in groups:
    for s in seeds:
        for c in conds:
            log(run_trial("main", c, s, a.height, a.pitch))
if "height" in groups:
    for h in (0.05, 0.15, 0.30, 0.50):
        for p in (0.0, 45.0, 90.0):
            for s in seeds[:3]:
                log(run_trial("height", "A", s, h, p))
if "repeat" in groups:
    for k in range(3):
        r = run_trial("repeat", "C", 0, a.height, a.pitch)
        r["repeat_index"] = k
        log(r)
if "highres" in groups:
    # 저해상도 height 묶음에서 splash 지표가 가장 작은/큰 조건 + main 의 기준점을 고해상도로
    hr = [r for r in results if r["group"] == "height" and not r["initial_overlap_invalid"]]
    if hr:
        key = lambda r: r["over_rim_mass_peak"] + r["p999_splash_height"]
        cand = sorted(hr, key=key)
        picks = [cand[0], cand[-1]]
        picks = [(r["release_height_m"], r["pitch_deg"]) for r in picks]
    else:
        picks = []
    picks.append((a.height, a.pitch))
    for h, p in dict.fromkeys(picks):
        log(run_trial("highres", "A", 0, h, p, voxel=0.0125, iters=400))

# ── 요약 ───────────────────────────────────────────────────────────────────────
summ = {"backend": "newton 1.8.0 MPM (isothermal water-like fluid, NOT molten metal) + MuJoCo rigid plate",
        "err_range": {k: v for k, v in ERR_RANGE.items()}, "pin_noise_m": PIN_NOISE, "seeds": seeds,
        "duration_s": a.duration, "main_height_m": a.height, "main_pitch_deg": a.pitch, "low_height_m": a.low_height,
        "groups": {}}
M = ["release_pos_err_mm", "release_ang_err_deg", "cumulative_escaped_mass_minus_baseline", "over_rim_mass_peak",
     "p999_splash_height", "max_splash_height", "fluid_kinetic_energy_peak", "above_surface_mass_peak"]
for g in ("main", "height", "repeat", "highres"):
    rs = [r for r in results if r["group"] == g]
    if not rs:
        continue
    by = {}
    for r in rs:
        k = r["condition"] if g == "main" else f"{r['condition']}_h{r['release_height_m']:.2f}_p{r['pitch_deg']:.0f}_dx{r['voxel_m']}"
        by.setdefault(k, []).append(r)
    summ["groups"][g] = {k: {"n": len(v), **{m: {"mean": float(np.mean([x[m] for x in v])),
                                                  "std": float(np.std([x[m] for x in v]))} for m in M},
                             "entry_wall_contact_frac": float(np.mean([x["entry_wall_contact"] for x in v])),
                             "wall_contact_any_time_frac": float(np.mean([x["wall_contact_any_time"] for x in v])),
                             "initial_overlap_frac": float(np.mean([x["initial_overlap_invalid"] for x in v])),
                             "plate_inserted_frac": float(np.mean([x["plate_inserted"] for x in v])),
                             "reachable_frac": float(np.mean([x["robot_reachable"] for x in v])),
                             "gripper_wet_frac": float(np.mean([x["collision_gripper_fluid"] for x in v]))}
                         for k, v in by.items()}
json.dump(summ, open(os.path.join(out, "summary.json"), "w"), indent=2, default=float)
print(json.dumps(summ["groups"].get("main", {}), indent=1, default=float)[:3000])
