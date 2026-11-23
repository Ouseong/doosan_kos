"""
가반하중·손목 토크·필요 파지력 점검 (PLAN §1.5). 재질 미확정이므로 밀도 후보마다 계산하고, 초과는 실험에서 빼지 않고 플래그만 단다.

  python isaac/furnace/sim/payload_check.py [--config ...] [--out output/furnace/payload_check.json]
"""

import argparse
import json
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def check(cfg, g=9.81):
    L, W, t = cfg["plate"]["size_m"]
    V = L * W * t
    mg = cfg["gripper"]["mass_kg"]
    payload = cfg["robot"]["payload_kg"]
    wrist = cfg["robot"]["wrist_effort_nm"]
    mu = cfg["gripper"]["grasp_mu"]
    npins = 15                                   # 기본 삽입 깊이에서 jaw 당 눌리는 핀 수 (3 행 × 5 열)
    edge = cfg["grasp"]["edge"]
    lever = W / 2 if edge == "long" else L / 2   # 파지 가장자리 → 판 무게중심
    out = []
    for rho in cfg["plate"]["density_candidates"]:
        m = rho * V
        total = m + mg
        F_need = m * g / (2 * mu)                # 판이 수직으로 매달릴 때 jaw 당 필요 법선력
        out.append({
            "density_candidate": rho, "plate_mass_kg": round(m, 3), "gripper_mass_kg_assumed": mg,
            "total_payload_kg": round(total, 3), "payload_limit_kg": payload,
            "payload_exceeded": total > payload,
            "lever_m": lever,
            "plate_torque_horizontal_Nm": round(m * g * lever, 2),          # 판을 수평으로 들 때 파지점 모멘트
            "wrist_effort_limit_Nm": wrist,
            "normal_force_per_jaw_needed_N": round(F_need, 1),
            "pin_k_min_at_4mm_N_per_m": round(F_need / (npins * 0.004), 0),
            "flag": ("ROBOT_INFEASIBLE(payload)" if total > payload else "ok"),
        })
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "..", "config", "default.yaml"))
    ap.add_argument("--out", default=os.path.join(ROOT, "output", "furnace", "payload_check.json"))
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    r = check(cfg)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump({"assumptions": {"gripper_mass_kg": cfg["gripper"]["mass_kg"], "mu": cfg["gripper"]["grasp_mu"],
                               "pins_pressed_per_jaw": 15, "note": "gripper mass and mu are [assumed]"},
               "candidates": r}, open(a.out, "w"), indent=2)
    for x in r:
        print(x)
