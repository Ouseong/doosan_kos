import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sim"))
from splash_metrics import CylinderVessel, SplashAccumulator, subtract_baseline  # noqa: E402

V = CylinderVessel(radius=0.4, rim_z=0.7, wall=0.05, tol=0.0)


def test_classify_regions():
    x = np.array([
        [0.0, 0.0, 0.2],     # inside
        [0.42, 0.0, 0.3],    # wall thickness → penetrated
        [0.0, 0.0, -0.02],   # floor thickness → penetrated
        [0.5, 0.0, 0.3],     # beyond outer wall → outside
        [0.41, 0.0, 0.75],   # over the rim, outside radius → outside
        [0.0, 0.0, 0.8],     # above rim but over the opening → inside
        [0.0, 0.0, -0.1],    # below floor → outside
    ])
    assert V.classify(x).tolist() == [0, 1, 1, 2, 2, 0, 2]


def test_particle_crossing_many_times_counted_once():
    m = np.array([1.0, 2.0])
    acc = SplashAccumulator(V, m, surface_z0=0.45)
    inside = np.array([0.0, 0.0, 0.3])
    out = np.array([0.6, 0.0, 0.3])
    for i in range(10):              # particle 0 goes in/out 5 times, particle 1 stays inside
        acc.update(i * 0.01, np.array([out if i % 2 else inside, inside]))
    s = acc.summary()
    assert s["cumulative_escaped_mass"] == pytest.approx(1.0)
    assert s["cumulative_escaped_count"] == 1
    assert s["final_outside_mass"] == pytest.approx(1.0)     # last frame i=9 is odd → outside


def test_final_vs_cumulative_differ_when_particle_returns():
    m = np.ones(1)
    acc = SplashAccumulator(V, m, surface_z0=0.45)
    acc.update(0.0, np.array([[0.6, 0.0, 0.3]]))
    acc.update(0.1, np.array([[0.0, 0.0, 0.3]]))
    s = acc.summary()
    assert s["final_outside_mass"] == 0.0
    assert s["cumulative_escaped_mass"] == 1.0


def test_penetration_not_counted_as_escape():
    acc = SplashAccumulator(V, np.ones(1), surface_z0=0.45)
    acc.update(0.0, np.array([[0.43, 0.0, 0.2]]))
    s = acc.summary()
    assert s["cumulative_escaped_mass"] == 0.0
    assert s["final_penetrated_mass"] == 1.0


def test_over_rim_and_height_and_ke():
    m = np.array([1.0, 1.0])
    acc = SplashAccumulator(V, m, surface_z0=0.45)
    acc.update(0.0, np.array([[0.0, 0.0, 0.45], [0.0, 0.0, 0.40]]), np.zeros((2, 3)))
    acc.update(0.1, np.array([[0.0, 0.0, 0.95], [0.6, 0.0, 0.40]]), np.array([[0, 0, 1.0], [2.0, 0, 0]]))
    s = acc.summary()
    assert s["over_rim_mass_peak"] == 1.0
    assert s["max_splash_height"] == pytest.approx(0.5)
    assert s["outside_kinetic_energy_peak"] == pytest.approx(0.5 * 1.0 * 4.0)


def test_baseline_subtraction_keeps_raw():
    r = {"final_outside_mass": 3.0, "cumulative_escaped_mass": 5.0, "max_splash_height": 0.2}
    b = {"final_outside_mass": 1.0, "cumulative_escaped_mass": 1.5, "max_splash_height": 0.01}
    o = subtract_baseline(r, b)
    assert o["final_outside_mass"] == 3.0
    assert o["final_outside_mass_minus_baseline"] == 2.0
    assert o["cumulative_escaped_mass_minus_baseline"] == 3.5


def test_above_surface_mass():
    acc = SplashAccumulator(V, np.ones(3), surface_z0=0.45, above_margin=0.02)
    acc.update(0.0, np.array([[0, 0, 0.44], [0, 0, 0.46], [0, 0, 0.50]]))
    assert acc.summary()["above_surface_mass_peak"] == 1.0
