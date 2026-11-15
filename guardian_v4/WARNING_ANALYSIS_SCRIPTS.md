# ⚠️ Warning: the v4 analysis and figure scripts are still the v3 versions

The **controller code itself is verified**. `v4_check.m` was run in GNU Octave 11.3 on 15 Nov 2026 and reproduces the Python prototype exactly: A/B/C each 10/10 hits, first hit 0.30 / 0.30 / 0.31 s.

These files were copied from v3 unchanged. **Do not use their output as "v4" results in the paper:**

| File | Problem |
|---|---|
| `make_bode.m` | Compares v3's old lead designs, not v3 against v4. |
| `margins_check.m` | Gives outer-loop margins only (Gc·Gp,Rv), never the full two-loop system or the real motor plant. |
| `plant_and_margins.m` | `C.numP/denP` is the plant with Rv folded in, so the "plant poles" and ζ it prints are the damped plant, not the physical one. |
| `make_figures.m` | Defaults to `{'track','ambush'}` modes; v4 uses `'ambush2'`. |
| `make_mc_figure.m`, `analyse_sweep.m`, `replay_fire.m`, `sweep_batch.m` | Legacy v2/v3 Monte-Carlo tools (track mode, `sweep/` data not included). |

## What to plot instead (e.g. in Python)
- **Physical plant:** Gp(s) = N·Kt / ( s[(J_eq s + b)(L s + R) + N²·Kt·Ke] ), with R = Rm = 1 Ω (ζ = 0.094 at 55.9 rad/s).
- **Plant seen by the outer loop:** the same formula with R = Rm + Rv = 6 Ω (Rv = 5 Ω virtual resistance).
- **Outer loop:** L(s) = Gc(s)·Gp,Rv(s). Expect PM ≈ 55°, GM ≈ 11.9 dB, ω_c = 15 rad/s, Ms ≈ 1.48.
- **True margins of the two-loop system** (broken at the motor input):
  L_in(s) = Gc(s)·Gp(s) + Rv·(J_eq s + b) / ((J_eq s + b)(L s + R) + N²·Kt·Ke).
- **Gc(s) polynomials:** `C.numC`, `C.denC` from `design_controller(guardian_params())`.
- **Python versions already exist:** `../overnight/gsim.py` (design) and `../overnight/discrete_analysis.py` (discrete closed-loop damping vs controller call period, as in `../overnight/figs/damping_vs_dt.png`).
