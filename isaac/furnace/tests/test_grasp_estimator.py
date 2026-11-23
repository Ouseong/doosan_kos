import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
import grasp_geometry as G  # noqa: E402
from grasp_estimator import estimate  # noqa: E402

PS = (0.5, 0.3, 0.01)
INS = 0.03


def obs(err, opening=0.0, noise=0.0, rng=None):
    T = G.plate_in_S(PS, INS, err)
    dp, dn, _, _ = G.pin_displacements(T, PS, opening)
    if noise:
        dp = np.where(dp > 0, dp + rng.normal(0, noise, 25), 0)
        dn = np.where(dn > 0, dn + rng.normal(0, noise, 25), 0)
    s = G.shift_from_opening(opening)
    return dp, dn, s, s


def test_nominal():
    e = estimate(*obs({}), INS, plate_thickness=0.01)
    assert e["ok"]
    assert abs(e["dx"]) < 1e-6 and abs(e["ry"]) < 1e-6 and abs(e["rz"]) < 1e-6
    assert abs(e["thickness_error"]) < 1e-6
    assert abs(e["dz"]) <= G.PITCH / 2 + 1e-9


@pytest.mark.parametrize("seed", range(20))
def test_random_errors(seed):
    rng = np.random.default_rng(seed)
    err = dict(dx=rng.uniform(-0.002, 0.002), dy=rng.uniform(-0.02, 0.02), dz=rng.uniform(-0.008, 0.008),
               rx=np.deg2rad(rng.uniform(-3, 3)), ry=np.deg2rad(rng.uniform(-3, 3)), rz=np.deg2rad(rng.uniform(-3, 3)))
    e = estimate(*obs(err, noise=1e-4, rng=rng), INS)
    assert e["ok"]
    # dx 는 관측 불가 dy 와 tan(rz)*dy 만큼 섞인다 (estimator docstring)
    assert abs(e["dx"] - err["dx"]) < 6e-4 + abs(np.tan(err["rz"]) * err["dy"]) + abs(np.tan(err["ry"])) * G.PITCH
    assert abs(e["ry"] - err["ry"]) < np.deg2rad(1.0)
    assert abs(e["rz"] - err["rz"]) < np.deg2rad(1.0)
    assert abs(e["dz"] - err["dz"]) < G.PITCH          # 양자화
    assert e["observable"]["dy"] is False and e["dy"] == 0.0
    assert e["observable"]["rx"] is False


def test_dy_does_not_change_observation():
    a = obs({})
    b = obs(dict(dy=0.03))
    assert np.allclose(a[0], b[0]) and np.allclose(a[1], b[1])


def test_no_gt_input():
    import inspect
    params = inspect.signature(estimate).parameters
    assert not any("pose" in p or "gt" in p for p in params)


def test_plane_offset_exact():
    rng = np.random.default_rng(3)
    err = dict(dx=0.001, dy=0.02, dz=0.004, rx=0.0, ry=np.deg2rad(2), rz=np.deg2rad(2))
    e = estimate(*obs(err), INS)
    a, b = -np.tan(err["rz"]), np.tan(err["ry"])
    c_true = err["dx"] - a * err["dy"] - b * err["dz"]
    assert abs(e["plane_offset_at_pin_center"] - c_true) < 2e-4
