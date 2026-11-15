# Guardian v3: predictive turret elevation control (ME319 Assignment 2)

v3 = v2 (`guardian_v2/proposed/`) + two compliance fixes from `AUDIT.md`:
1. **Default mode is `ambush`** (aim at the elevation predicted at t + tp = 0.5 s, hold, fire as the target crosses). This is the mode that uses the prescribed horizon tp as Sec. 4 of the brief requires. `track` stays available for comparison.
2. **The generator enforces the operating envelope and blackout bounds** (error, not warning), as Sec. 3 requires.

Chain: radar (r, θ) → constant-acceleration EKF → prediction to t + tp → aim-point elevation reference → reference shaper → notch·PI·lead Gc(s) + inverse-plant feedforward → turret. Firing is covariance-gated: fire when miss_est + 3σ⊥ ≤ 2 m.

## Files
| File | Role |
|---|---|
| `turret_plant.m`, `turret_controller.m`, `target_motion_generator.m` | the three required interfaces |
| `guardian_params.m` | every physical and design parameter (single source of truth) |
| `design_controller.m` | Gp(s), Gc(s) (lead sized from PM*), state-space, feedforward gains |
| `run_scenario.m`, `runmode.m` | closed-loop simulation and scoring: `R = run_scenario('A')`, `runmode('B','track')` |
| `margins_check.m`, `make_bode.m`, `plant_and_margins.m` | margins/Bode/root locus (Octave `control` package) |
| `make_figures.m` | scenario figures → `figs/` |
| `sweep_batch.m`, `analyse_sweep.m`, `replay_fire.m`, `make_mc_figure.m` | v1 Monte-Carlo tooling (R1–R100) |
| `realism/` | realistic-conditions harness and the exact runs behind `REALISM.md` (also contains copies of the v2 stress tools) |
| `AUDIT.md` | requirement-by-requirement compliance check |
| `REALISM.md` | assigned vs realistic parameters, research sources, stress results |
| `IDEAS.md` | prioritised to-do list and improvement ideas |

## Key numbers (v3 design)
- Gp(s) = 3.125 / [s (s² + 10.47 s + 3129.7)]: poles 0, −5.23 ± 55.7j (ζ = 0.094)
- Gc: notch on the resonant pair (ζ → 0.7), PI zero 5 rad/s, lead a = 1.24 → ωc = 10 rad/s, **PM 55°, GM 17.3 dB, Ms 1.29**
- EKF: q_jerk = 20², n_init = 6; fire gate k = 3; reference shaper 15 rad/s, 20 rad/s²
- A/B/C (ambush): **8/8 hits each, first hit 0.44 s**. 160 hard stress cases: **100 % intercepted, mean first hit 0.63 s, 99.9 % of shots hit**

## Reproduce (Octave 8 or MATLAB)
```matlab
R = run_scenario('C');                        % default mode (ambush)
R = runmode('C','track');                      % comparison mode
margins_check();                               % Octave: pkg load control
make_figures({'A','B','C'},{'ambush','track'}); make_bode   % figures -> figs/
```
Headless Octave figures: `xvfb-run -a octave-cli --eval "make_figures; make_bode"`.
Realistic stress tests: see `realism/` and the "Reproduce" section of `REALISM.md`. Build the hard cases first with `realism/stress_case.m`.

Note: the controller keeps state in persistent variables. It resets at t = 0 or on a backward time jump > 0.5 s; otherwise call `clear turret_controller` between runs.
