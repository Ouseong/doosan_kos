"""
Splash 지표 (순수 numpy, Isaac/Newton 불필요). PLAN Phase 2, SPEC §14.

영역 판정 (원통 용기: 내경 반지름 R, 벽 두께 T, 바닥 z=0, rim 높이 z_rim):
  inside       r <= R 이고 z >= -tol
  penetrated   R < r <= R+T 이고 z <= z_rim  (벽 두께 안)   또는  r <= R 이고 z < -tol (바닥 안)
               → 수치적 관통. splash 가 아니다. 따로 보고한다.
  outside      그 밖 전부: r > R+T, 또는 r > R 이고 z > z_rim (rim 을 넘어간 것), 또는 z < -T (바닥 아래)

집계 규칙:
  - final_outside_mass: 마지막 프레임에 outside 인 질량
  - cumulative_escaped_mass: 한 번이라도 outside 였던 "고유 입자" 질량. 입자 ID 당 1 회만 센다
    (나갔다 들어왔다를 반복해도 중복 집계하지 않는다)
  - over_rim_mass_peak: z > z_rim 인 질량의 시간 최대값 (안/밖 무관)
  - max_splash_height: 초기 액면 기준 최고 입자 높이. 단일 입자 이상치를 가리려고 상위 분위수(p99.9, p99)도 같이
  - outside_kinetic_energy_peak: outside 입자 운동에너지의 시간 최대값
  - fluid_kinetic_energy_peak: 전체 유체 운동에너지 시간 최대값 (판이 액체를 움직였는지)
  - above_surface_mass_peak: 초기 액면 + above_margin 위에 있는 질량의 시간 최대값 (판이 액체를 밀어 올린 양)
  - 시각 효과 입자는 넣지 않는다 (입력은 물리 입자만)
  - leak baseline: 같은 설정 no-impact 결과를 subtract_baseline() 으로 빼서 원값·차감값 둘 다 보고
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class CylinderVessel:
    radius: float          # 내경 반지름 R [m]
    rim_z: float           # rim 높이 [m] (바닥 z=0 기준)
    wall: float            # 벽·바닥 두께 T [m]
    tol: float = 0.0       # 바닥 판정 여유 [m]
    center_xy: tuple[float, float] = (0.0, 0.0)

    def classify(self, x: np.ndarray) -> np.ndarray:
        """x (N,3) → 0 inside, 1 penetrated, 2 outside."""
        r = np.hypot(x[:, 0] - self.center_xy[0], x[:, 1] - self.center_xy[1])
        z = x[:, 2]
        R, T = self.radius, self.wall
        inside = (r <= R) & (z >= -self.tol)
        pen = (~inside) & (((r > R) & (r <= R + T) & (z <= self.rim_z) & (z >= -T))
                           | ((r <= R) & (z < -self.tol) & (z >= -T)))
        out = ~(inside | pen)
        return np.where(inside, 0, np.where(pen, 1, 2)).astype(np.int8)


@dataclass
class SplashAccumulator:
    vessel: CylinderVessel
    mass: np.ndarray                 # (N,) 입자 질량
    surface_z0: float                # 초기 액면 높이
    above_margin: float = 0.02       # above_surface_mass 판정 여유 [m]
    ever_out: np.ndarray = field(init=False)
    series: dict = field(init=False)

    def __post_init__(self):
        self.mass = np.asarray(self.mass, dtype=np.float64)
        self.ever_out = np.zeros(self.mass.shape[0], dtype=bool)
        self.series = {k: [] for k in ("t", "outside_mass", "penetrated_mass", "over_rim_mass",
                                       "max_z", "p999_z", "p99_z", "outside_ke", "above_surface_mass", "fluid_ke")}

    def update(self, t: float, x: np.ndarray, v: np.ndarray | None = None):
        c = self.vessel.classify(x)
        out = c == 2
        self.ever_out |= out
        m = self.mass
        s = self.series
        s["t"].append(float(t))
        s["outside_mass"].append(float(m[out].sum()))
        s["penetrated_mass"].append(float(m[c == 1].sum()))
        s["over_rim_mass"].append(float(m[x[:, 2] > self.vessel.rim_z].sum()))
        z = x[:, 2]
        s["above_surface_mass"].append(float(m[z > self.surface_z0 + self.above_margin].sum()))
        s["max_z"].append(float(z.max()))
        s["p999_z"].append(float(np.percentile(z, 99.9)))
        s["p99_z"].append(float(np.percentile(z, 99.0)))
        s["fluid_ke"].append(float(0.5 * (m * (v ** 2).sum(1)).sum()) if v is not None else 0.0)
        if v is not None and out.any():
            s["outside_ke"].append(float(0.5 * (m[out] * (v[out] ** 2).sum(1)).sum()))
        else:
            s["outside_ke"].append(0.0)

    def summary(self) -> dict:
        s = self.series
        if not s["t"]:
            raise ValueError("no frames")
        return {
            "total_mass": float(self.mass.sum()),
            "final_outside_mass": s["outside_mass"][-1],
            "cumulative_escaped_mass": float(self.mass[self.ever_out].sum()),
            "cumulative_escaped_count": int(self.ever_out.sum()),
            "final_penetrated_mass": s["penetrated_mass"][-1],
            "penetrated_mass_peak": max(s["penetrated_mass"]),
            "over_rim_mass_peak": max(s["over_rim_mass"]),
            "max_splash_height": max(s["max_z"]) - self.surface_z0,
            "p999_splash_height": max(s["p999_z"]) - self.surface_z0,
            "p99_splash_height": max(s["p99_z"]) - self.surface_z0,
            "outside_kinetic_energy_peak": max(s["outside_ke"]),
            "above_surface_mass_peak": max(s["above_surface_mass"]),
            "fluid_kinetic_energy_peak": max(s["fluid_ke"]),
        }


MASS_KEYS = ("final_outside_mass", "cumulative_escaped_mass", "over_rim_mass_peak",
             "final_penetrated_mass", "penetrated_mass_peak")


def subtract_baseline(result: dict, baseline: dict) -> dict:
    """no-impact baseline 을 뺀 값. 원값은 그대로 두고 *_minus_baseline 키를 추가한다."""
    out = dict(result)
    for k in MASS_KEYS + ("max_splash_height", "p999_splash_height", "p99_splash_height",
                          "outside_kinetic_energy_peak", "above_surface_mass_peak", "fluid_kinetic_energy_peak"):
        if k in result and k in baseline:
            out[f"{k}_minus_baseline"] = result[k] - baseline[k]
    return out
