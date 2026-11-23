"""
evaluate 결과(results.csv) → 비교 그림 2 장 + 표(md). 시뮬레이션 데이터 플롯.

  ~/venvs/furnace_newton/bin/python isaac/furnace/make_results_plots.py output/furnace/evaluate/phase6
"""
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

d = sys.argv[1]
R = list(csv.DictReader(open(f"{d}/results.csv")))
for r in R:
    for k, v in list(r.items()):
        try:
            r[k] = float(v)
        except (TypeError, ValueError):
            r[k] = {"True": True, "False": False}.get(v, v)

M = [("release_pos_err_mm", "release position error [mm]"), ("release_ang_err_deg", "release angle error [deg]"),
     ("p999_splash_height", "p99.9 splash height above surface [m]"), ("fluid_kinetic_energy_peak", "fluid KE peak [J]"),
     ("over_rim_mass_peak", "over-rim mass peak [kg]"), ("cumulative_escaped_mass_minus_baseline", "escaped mass - baseline [kg]")]
main = [r for r in R if r["group"] == "main"]
fig, axs = plt.subplots(2, 3, figsize=(13, 7))
conds = ["A", "B", "C", "D"]
lines = ["| metric | " + " | ".join(conds) + " |", "|---|" + "---|" * len(conds)]
for ax, (k, lab) in zip(axs.ravel(), M):
    data = [[r[k] for r in main if r["condition"] == c] for c in conds]
    ax.boxplot(data, tick_labels=conds, showmeans=True)
    ax.set_title(lab, fontsize=9)
    lines.append(f"| {k} | " + " | ".join(f"{np.mean(x):.4g} ± {np.std(x):.2g}" if x else "-" for x in data) + " |")
for k in ("entry_wall_contact", "wall_contact_any_time", "initial_overlap_invalid", "robot_reachable", "collision_gripper_fluid", "plate_inserted"):
    lines.append(f"| {k} (frac) | " + " | ".join(f"{np.mean([float(r[k]) for r in main if r['condition'] == c]):.2f}" for c in conds) + " |")
fig.suptitle("Phase 6 main: A fixed / B pin-estimate corrected / C oracle(GT) / D low height (A at 0.06 m). "
             "Newton MPM water-like fluid, dx 25 mm. Simulation data plot.", fontsize=9)
fig.tight_layout(); fig.savefig(f"{d}/main_conditions.png", dpi=100)

hr = [r for r in R if r["group"] == "height" and not r["initial_overlap_invalid"]]
if hr:
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.8))
    for ax, (k, lab) in zip(axs, [M[2], M[3], M[4]]):
        for p in sorted({r["pitch_deg"] for r in hr}):
            hs = sorted({r["release_height_m"] for r in hr})
            ys = [np.mean([r[k] for r in hr if r["pitch_deg"] == p and r["release_height_m"] == h]) for h in hs]
            es = [np.std([r[k] for r in hr if r["pitch_deg"] == p and r["release_height_m"] == h]) for h in hs]
            ax.errorbar(hs, ys, yerr=es, marker="o", capsize=3, label=f"pitch {p:.0f}°")
        ax.set_xlabel("release height of lowest plate point above surface [m]"); ax.set_title(lab, fontsize=9)
        ax.legend(fontsize=7)
    fig.suptitle("Phase 6 height/pitch sweep (condition A, 3 seeds each, dx 25 mm). pitch 0 = vertical plate edge-first, "
                 "90 = horizontal plate. Simulation data plot.", fontsize=9)
    fig.tight_layout(); fig.savefig(f"{d}/height_pitch_sweep.png", dpi=100)
    lines += ["", "| height [m] | pitch [deg] | n | centre offset [m] | reachable | p999 height [m] | KE peak [J] | over-rim [kg] | escaped-baseline [kg] |",
              "|---|---|---|---|---|---|---|---|---|"]
    for h in sorted({r["release_height_m"] for r in hr}):
        for p in sorted({r["pitch_deg"] for r in hr}):
            s = [r for r in hr if r["release_height_m"] == h and r["pitch_deg"] == p]
            lines.append(f"| {h:.2f} | {p:.0f} | {len(s)} | {np.mean([r['target_center_offset_m'] for r in s]):.3f} | {np.mean([float(r['robot_reachable']) for r in s]):.2f} | "
                         f"{np.mean([r['p999_splash_height'] for r in s]):.4f} | {np.mean([r['fluid_kinetic_energy_peak'] for r in s]):.2f} | "
                         f"{np.mean([r['over_rim_mass_peak'] for r in s]):.3f} | {np.mean([r['cumulative_escaped_mass_minus_baseline'] for r in s]):.3f} |")
for g in ("repeat", "highres"):
    s = [r for r in R if r["group"] == g]
    if s:
        lines += ["", f"**{g}**", "", "| height | pitch | dx | p999 height | KE peak | over-rim | escaped-baseline |", "|---|---|---|---|---|---|---|"]
        for r in s:
            lines.append(f"| {r['release_height_m']:.2f} | {r['pitch_deg']:.0f} | {r['voxel_m']} | {r['p999_splash_height']:.4f} | "
                         f"{r['fluid_kinetic_energy_peak']:.2f} | {r['over_rim_mass_peak']:.3f} | {r['cumulative_escaped_mass_minus_baseline']:.3f} |")
open(f"{d}/tables.md", "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
