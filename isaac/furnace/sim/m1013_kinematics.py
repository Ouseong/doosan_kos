"""
M1013 정기구학/역기구학 (순수 numpy + scipy). 관절 원점·축은 공식 URDF 에서 직접 읽는다.

- FK: base_link → link_6 (플랜지) 4×4
- IK: 수치 최적화 (scipy least_squares, 관절 한계 안). 여러 초기값에서 시작해 가장 좋은 해
- Isaac 에서 읽은 link_6 위치와 대조 검증 (Phase 0: 홈 자세 link_6 = (-0.559, 0.0345, 0.6514))
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
URDF = os.path.join(ROOT, "assets_external/doosan-robot2/dsr_description2/urdf/m1013.urdf")


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


class M1013:
    def __init__(self, urdf=URDF):
        tree = ET.parse(urdf).getroot()
        self.joints = []
        for j in tree.findall("joint"):
            if j.get("type") != "revolute":
                continue
            o = j.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in o.get("rpy", "0 0 0").split()])
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            lim = j.find("limit")
            T = np.eye(4); T[:3, :3] = rpy_matrix(*rpy); T[:3, 3] = xyz
            self.joints.append({"name": j.get("name"), "T": T, "axis": ax,
                                "lower": float(lim.get("lower")), "upper": float(lim.get("upper")),
                                "effort": float(lim.get("effort")), "velocity": float(lim.get("velocity"))})
        assert [j["name"] for j in self.joints] == [f"joint_{i}" for i in range(1, 7)]
        self.lower = np.array([j["lower"] for j in self.joints])
        self.upper = np.array([j["upper"] for j in self.joints])

    def fk(self, q, base=np.eye(4), all_frames=False):
        T = base.copy()
        frames = []
        for j, qi in zip(self.joints, q):
            R = np.eye(4); R[:3, :3] = axis_angle(j["axis"], qi)
            T = T @ j["T"] @ R
            frames.append(T.copy())
        return frames if all_frames else T

    def ik(self, T_target, q0=None, base=np.eye(4), tool=np.eye(4), seeds=12, rng=0, w_rot=0.3,
           q_lim_margin=0.05, extra_cost=None):
        """T_target: tool 프레임의 목표 world 자세. tool = link_6 → 툴 변환. 반환 (q, pos_err_m, rot_err_rad)."""
        from scipy.optimize import least_squares
        lo = self.lower + q_lim_margin
        hi = self.upper - q_lim_margin

        def res(q):
            T = self.fk(q, base) @ tool
            e_p = T[:3, 3] - T_target[:3, 3]
            Re = T[:3, :3].T @ T_target[:3, :3]
            ang = np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1))
            v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
            n = np.linalg.norm(v)
            e_r = (v / n * ang) if n > 1e-9 else np.zeros(3)
            r = np.concatenate([e_p, w_rot * e_r])
            if extra_cost is not None:
                r = np.concatenate([r, extra_cost(q)])
            return r

        g = np.random.default_rng(rng)
        starts = [] if q0 is None else [np.clip(q0, lo, hi)]
        starts += [g.uniform(np.maximum(lo, -np.pi), np.minimum(hi, np.pi)) for _ in range(seeds)]
        best = None
        for s in starts:
            sol = least_squares(res, s, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=400)
            T = self.fk(sol.x, base) @ tool
            pe = float(np.linalg.norm(T[:3, 3] - T_target[:3, 3]))
            Re = T[:3, :3].T @ T_target[:3, :3]
            re = float(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)))
            cost = pe + 0.1 * re + (0.0 if q0 is None else 0.01 * np.linalg.norm(sol.x - q0))
            if best is None or cost < best[3]:
                best = (sol.x, pe, re, cost)
        return best[0], best[1], best[2]


def make_T(pos, R=np.eye(3)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = pos
    return T


if __name__ == "__main__":
    r = M1013()
    q_home = np.deg2rad([0, 0, -90, 0, -90, 0])
    print("home link_6:", np.round(r.fk(q_home)[:3, 3], 4), "(Isaac Phase 0: -0.559, 0.0345, 0.6514)")
    T = r.fk(np.deg2rad([20, 10, -100, 30, -60, 45]))
    q, pe, re = r.ik(T)
    print("ik roundtrip err", pe, re)
