# Guardian — predictive turret elevation control (ME319 Assignment 2)

MATLAB/Octave implementation of a predictive-feedback fire-control loop:
radar (r, θ) → constant-acceleration EKF → prediction to t + tp → elevation reference → notch-PI-lead compensator + feedforward → turret.

## Files
| File | Role |
|---|---|
| `turret_plant.m` | Required interface. First-principles DC-motor + 100:1 gearbox elevation drive, end-stops. |
| `turret_controller.m` | Required interface. EKF, predictor, reference generator, Gc(s), covariance-gated firing. |
| `target_motion_generator.m` | Required interface. Scenarios `'A'`, `'B'`, `'C'`, and randomised `'R<n>'` test cases. |
| `guardian_params.m` | All physical and design parameters in one place. |
| `design_controller.m` | Gp(s), Gc(s) polynomials and state-space (no toolbox needed). |
| `run_scenario.m` | Closed-loop simulation (RK4, 1 ms) and engagement scoring. |
| `plant_and_margins.m` | Bode, margins, root locus, step/ramp (needs Control toolbox). |
| `runmode.m` | `runmode(scenario, 'track'|'ambush', verbose, overrides)` wrapper around `run_scenario`. |
| `sweep_batch.m`, `launch.sh` | Monte-Carlo batch over `R<n>` cases (2 parallel Octave workers). |
| `replay_fire.m`, `analyse_sweep.m` | Offline fire-gate replay (track mode) and Monte-Carlo metrics. |
| `margins_check.m`, `make_bode.m` | GM/PM/Ms of the designed loop; Bode figure old vs new. |
| `make_figures.m`, `make_mc_figure.m` | Scenario figures and Monte-Carlo summary (`figs/`). |
| `RESULTS.md` | Debug log, compensator fix, tuning tables, final results. |

## Reproduce
```matlab
R = run_scenario('A');          % also 'B', 'C', 'R1'...'R100'
R = runmode('C', 'ambush');     % choose fire-control mode
```
Octave: `pkg load control` only for the analysis scripts. Headless figures: `xvfb-run -a octave-cli --eval "make_figures"`.

Status: v1 — runs cleanly in Octave 8.4; tuned on R1–R100, validated on R101–R200. See RESULTS.md.
