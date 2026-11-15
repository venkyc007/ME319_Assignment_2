# Guardian v2: stress test and proposed changes

## proposed/: drop-in replacement for the repo
Same files as v1. Only `turret_controller.m` and `guardian_params.m` changed:

| Change | Where | Why |
|---|---|---|
| wc 20 -> 10 rad/s | guardian_params | Stable for control steps up to 50 ms and for +-20 % J/L/Kt errors. GM 11.4 -> 17.3 dB, Ms 1.64 -> 1.29. |
| z_i 2 -> 5 rad/s | guardian_params | Removes the slow -2.2 rad/s closed-loop tail that delayed the first hit. PM is still 55 deg (lead a = 1.24). |
| 2-DOF reference shaper (ref_wf = 15 rad/s, 20 rad/s^2) | turret_controller | Critically damped acquisition: no overshoot into the 85 deg stop, peak voltage 4.7 kV -> 1.3 kV. |
| A new radar sample = valid flag AND a changed value | turret_controller | Works whether the evaluator pulses meas_valid or holds it high. Also checks theta for NaN. |
| Reset only at t = 0 or a large backward time jump; ignore small backward steps | turret_controller | Survives ODE-solver trial steps. |
| mode 'hybrid' and cross_fire options (off by default) | turret_controller | Tested, not adopted. |

Results with proposed/: A, B and C all 10/10, first hit 0.56 s.

## stress_test/
* `stress_case(id)`: 160 admissible cases. 1-40 weave, 41-80 jink, 81-110 close/crossing, 111-140 adversarial (reversals inside the blackouts), 141-160 high elevation. |a| goes up to 20 m/s^2, jerk up to 365 m/s^3, and some cases have 1-3 blackouts.
* `eval_stress(C, opts)`: an independent harness. It varies the step size (dt), the measurement convention (pulse, hold or NaN), the integrator, the initial angle and plant mismatch.
* `run2.sh tag "struct(...)" ids`: runs 2 parallel Octave workers. `summarize(tag)` and `export_csv` produce the metrics.
* `design_scan.m`, `dt_scan.m`, `mm_scan.m`: margins, sampled-data stability and mismatch stability (no toolbox needed).

Build the cases once:
`CS=cell(1,160); for k=1:160, CS{k}=stress_case(k); end; save('-v7','stress/cases.mat','CS')`
