#!/usr/bin/env python3
"""
FK 검증: m1013_with_hammer.urdf 로드 → 입력 관절각(deg)에서 hammer_strike world pose 출력.

usage:
    python verify_fk.py                       # q=0 (home) 에서 FK
    python verify_fk.py 0 -30 90 0 60 -98     # 임의 관절각 (deg)

검증 기준:
- link_6 → hammer_strike 까지의 link_6-frame 오프셋이 (0, -0.203, -0.027) m 이어야 함
  (gripper +(0,0.012,0) + hammer +(0,-0.128,0) + strike +(0,-0.087,-0.027), Rx 180 은 자식 frame 회전)
- hammer_strike 의 world pos 가 ABOVE_NAIL 자세에서 못 (-0.65,-0.45) 부근, z 0.10~0.25 사이에 위치
"""

import sys
import numpy as np
import pinocchio as pin

URDF = "/home/kos/Desktop/Code/doosan_kos/usd/m1013/m1013_with_hammer.urdf"
NAIL_BASE  = np.array([-0.65, -0.45, 0.05])
NAIL_HEAD  = np.array([-0.65, -0.45, 0.10])   # base + height 0.05


def fk(q_deg):
    model = pin.buildModelFromUrdf(URDF)
    data  = model.createData()
    q = np.deg2rad(np.asarray(q_deg, dtype=float))
    assert q.shape == (model.nq,), f"q must be {model.nq}-vector, got {q.shape}"

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    def fid(name): return model.getFrameId(name)
    def world(name):
        T = data.oMf[fid(name)]
        return T.translation, T.rotation

    p_link6, R_link6 = world("link_6")
    p_grip,  _       = world("gripper_full_link")
    p_hl,    _       = world("hammer_link")
    p_str,   R_str   = world("hammer_strike")

    # link_6 -> hammer_strike (in link_6 frame) — should be (0, -0.203, -0.027)
    T_link6 = data.oMf[fid("link_6")]
    T_str   = data.oMf[fid("hammer_strike")]
    T_rel   = T_link6.inverse() * T_str
    rel_pos = T_rel.translation
    rel_R   = T_rel.rotation

    print(f"q (deg)               = {[round(float(x),2) for x in q_deg]}")
    print()
    print(f"link_6 world pos      = {p_link6.round(4)}")
    print(f"gripper world pos     = {p_grip.round(4)}")
    print(f"hammer_link world pos = {p_hl.round(4)}")
    print(f"hammer_strike world   = {p_str.round(4)}    ← 핵심: 못 머리 좌표와 비교")
    print()
    print(f"link_6 → hammer_strike (link_6 frame):")
    print(f"  position = {rel_pos.round(4)}    (예상: [ 0.    -0.203 -0.027])")
    print(f"  rotation =")
    for row in rel_R.round(3):
        print(f"    {row}")
    print()
    print(f"nail base = {NAIL_BASE},  nail head = {NAIL_HEAD}")
    d = np.linalg.norm(p_str - NAIL_HEAD)
    print(f"|hammer_strike - nail head| = {d*100:.1f} cm")


if __name__ == "__main__":
    if len(sys.argv) == 1:
        q = [0.0] * 6
    elif len(sys.argv) == 7:
        q = [float(x) for x in sys.argv[1:]]
    else:
        sys.exit("usage: verify_fk.py [J1 J2 J3 J4 J5 J6  (deg)]")
    fk(q)
