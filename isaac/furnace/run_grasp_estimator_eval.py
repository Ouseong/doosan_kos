"""
Phase 5 평가: 핀 변위 → 파지 자세 추정기.
 1) 관측 가능성: 명목 파지에서 핀 변위의 6 자유도 야코비안 SVD (수치 미분)
 2) 합성 데이터: 무작위 파지 오차 N 개 → 추정 오차 표 (GT 는 평가에만 사용)
 3) Phase 4 물리 로그(robot_charge_demo) 의 hold 구간 핀 변위 → 추정 vs GT (판 중심의 S 좌표)

  ~/venvs/furnace_newton/bin/python isaac/furnace/run_grasp_estimator_eval.py
"""

import csv
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(HERE, "sim"))
import grasp_geometry as G  # noqa: E402
from grasp_estimator import estimate  # noqa: E402

cfg = yaml.safe_load(open(os.path.join(HERE, "config", "default.yaml")))
PS = tuple(cfg["plate"]["size_m"])
INS = cfg["grasp"]["insert_depth_m"]
out = os.path.join(ROOT, "output", "furnace", "grasp_estimator")
os.makedirs(out, exist_ok=True)
DOF = ["dx", "dy", "dz", "rx", "ry", "rz"]


def pins(err, opening=0.0):
    dp, dn, _, _ = G.pin_displacements(G.plate_in_S(PS, INS, err), PS, opening)
    return np.r_[dp, dn]


# 1) 야코비안 (작은 회전에서 dz 는 행 경계가 불연속이라 미분 0 일 수 있다 → 유한 차분 크기 두 가지)
rep = {"jacobian": {}}
for h_t, h_r, label in ((1e-4, 1e-3, "small"), (3e-3, 2e-2, "large")):
    J = []
    base = pins({})
    for k in DOF:
        h = h_t if k[0] == "d" else h_r
        J.append((pins({k: h}) - pins({k: -h})) / (2 * h))
    J = np.array(J).T                                    # (50, 6)
    sv = np.linalg.svd(J, compute_uv=False)
    rep["jacobian"][label] = {"column_norm": {k: float(np.linalg.norm(J[:, i])) for i, k in enumerate(DOF)},
                              "singular_values": sv.round(6).tolist(), "rank(>1e-6)": int((sv > 1e-6).sum())}

# 2) 합성
rng = np.random.default_rng(0)
N = 500
rows = []
for n in range(N):
    err = dict(dx=rng.uniform(-0.002, 0.002), dy=rng.uniform(-0.02, 0.02), dz=rng.uniform(-0.008, 0.008),
               rx=np.deg2rad(rng.uniform(-3, 3)), ry=np.deg2rad(rng.uniform(-3, 3)), rz=np.deg2rad(rng.uniform(-3, 3)))
    T = G.plate_in_S(PS, INS, err)
    dp, dn, _, _ = G.pin_displacements(T, PS, 0.0)
    dp = np.where(dp > 0, dp + rng.normal(0, 1e-4, 25), 0)     # 핀 변위 측정 잡음 0.1 mm [가정]
    dn = np.where(dn > 0, dn + rng.normal(0, 1e-4, 25), 0)
    s = G.shift_from_opening(0.0)
    e = estimate(dp, dn, s, s, INS, plate_thickness=PS[2])
    r = {"n": n, **{f"true_{k}": err[k] for k in DOF}, **{f"est_{k}": e.get(k, np.nan) for k in DOF},
         **{f"obs_{k}": e["observable"][k] for k in DOF}}
    rows.append(r)
with open(os.path.join(out, "synthetic.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
tab = {}
for k in DOF:
    t = np.array([r[f"true_{k}"] for r in rows]); e_ = np.array([r[f"est_{k}"] for r in rows])
    o = np.array([r[f"obs_{k}"] for r in rows])
    scale = 1000 if k[0] == "d" else np.degrees(1)
    err_ = (e_ - t) * scale
    tab[k] = {"unit": "mm" if k[0] == "d" else "deg", "true_range": [float(t.min() * scale), float(t.max() * scale)],
              "rmse": float(np.sqrt(np.nanmean(err_ ** 2))), "max_abs": float(np.nanmax(np.abs(err_))),
              "flagged_observable_frac": float(o.mean()),
              "prior_rmse(if_ignored)": float(np.sqrt(np.mean((t * scale) ** 2)))}
rep["synthetic"] = {"N": N, "noise_mm": 0.1, "error_ranges": "dx±2mm dy±20mm dz±8mm rx,ry,rz±3deg", "table": tab}

# 3) Phase 4 물리 로그
phys = {}
logp = os.path.join(ROOT, "output", "furnace", "robot_charge_demo", "friction_k5000", "log.csv")
if os.path.exists(logp):
    L = list(csv.DictReader(open(logp)))
    s0 = G.shift_from_opening(cfg["gripper"]["max_opening_m"])
    Tn = G.nominal_plate_in_S(PS, INS)
    out_rows = []
    for ph in ("close", "lift", "transit", "hold"):
        r = [x for x in L if x["phase"] == ph][-1]
        dp = np.array([float(r[f"pin_p{i:02d}"]) for i in range(25)])
        dn = np.array([float(r[f"pin_n{i:02d}"]) for i in range(25)])
        sp = s0 + float(r["jaw_p"]); sn = s0 - float(r["jaw_n"])
        e = estimate(dp, dn, sp, sn, INS, plate_thickness=PS[2])
        gt = np.array([float(r[f"gt_plate_in_S_{c}"]) for c in "xyz"])
        gt_err = gt - Tn[:3, 3]            # 판 중심의 명목 대비 이동 (S). 작은 회전이면 dx,dy,dz 와 같다
        out_rows.append({"phase": ph, "pins_p": int((dp > 2e-4).sum()), "pins_n": int((dn > 2e-4).sum()),
                         "jaw_shift_p_mm": sp * 1000, "jaw_shift_n_mm": sn * 1000,
                         "est_dx_mm": e.get("dx", np.nan) * 1000, "gt_dx_mm": gt_err[0] * 1000,
                         "est_dz_mm": e.get("dz", np.nan) * 1000, "gt_dz_mm": gt_err[2] * 1000,
                         "est_dy_mm(prior)": 0.0, "gt_dy_mm": gt_err[1] * 1000,
                         "est_ry_deg": np.degrees(e.get("ry", np.nan)), "est_rz_deg": np.degrees(e.get("rz", np.nan)),
                         "est_thickness_mm": e.get("thickness", np.nan) * 1000})
    phys = {"source": os.path.relpath(logp, ROOT), "rows": out_rows,
            "note": "GT = plate centre in gripper frame minus nominal; rotations GT not logged (small)"}
rep["physics_phase4"] = phys
json.dump(rep, open(os.path.join(out, "estimator_eval.json"), "w"), indent=2, default=float)
print(json.dumps(rep, indent=2, default=float))
