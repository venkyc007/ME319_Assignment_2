# Guardian: predictive turret elevation control (ME319 Assignment 2)

A laser turret must hit a manoeuvring drone 800–2000 m away. It uses a noisy 20 Hz radar that can go blind for up to 0.5 s. The loop is:

1. A Kalman filter estimates the target's motion.
2. The predictor works out where the target will be at **tp = 0.5 s**.
3. The turret is driven to that aim point by an LTI feedback controller.
4. The laser fires when the filter is confident the beam will be within 2 m.

## ✅ Final version: [`guardian_v4/`](guardian_v4/)

| Folder | What it is |
|---|---|
| **[`guardian_v4/`](guardian_v4/)** | **FINAL submission code** (MATLAB/Octave). Verified in GNU Octave 11.3: `v4_check.m` reproduces the Python prototype exactly. ⚠️ Read [`guardian_v4/WARNING_ANALYSIS_SCRIPTS.md`](guardian_v4/WARNING_ANALYSIS_SCRIPTS.md): its Bode, margin and figure scripts are still the v3 versions. |
| [`overnight/`](overnight/) | Python analysis: a simulator that reproduces v3 exactly, plus every option tested, 400 hard test cases, experiments, logs and figures. Start with [`REPORT.md`](overnight/REPORT.md) (what changed in v4 and why) and [`COMPARISON_v3_v4.md`](overnight/COMPARISON_v3_v4.md) (v3 vs v4, in the brief's world and on realistic hardware). |
| [`paper/`](paper/) | IEEE paper (`main.tex`, currently an outline). |
| [`archive/`](archive/) | Earlier versions, kept for history: `v1/`, `guardian_v2/`, `guardian_v3/` (previous final candidate: audit, realism study and ideas), plus `notes/` (old hand-off notes). |

## Run it
Run in MATLAB, or Octave ≥ 8, from inside `guardian_v4/`:
```matlab
v4_check                 % scenarios A, B, C, R1, R7 with expected results in the header
R = run_scenario('B');   % one scenario (RK4, 1 ms); also 'A', 'C', 'R<n>'
```
The required interfaces are `target_motion_generator.m`, `turret_controller.m` and `turret_plant.m`. Every parameter is set in `guardian_params.m`, and each v4 change there is a separate switch.

Python analysis (needs numpy, scipy and matplotlib), from `overnight/`:
```
python exp_round3.py     # v3 vs v4 on held-out hard cases and off-nominal conditions
python exp_realism.py    # realistic-hardware study
```
The first run builds a cache of test cases (`overnight/case_cache/`, about 360 MB, git-ignored).

## Results (Octave, 1 ms)
| | v3 (archive) | **v4 (final)** |
|---|---|---|
| Scenarios A / B / C | 8/8 hits each, first hit 0.44 s | **10/10 each, first hit 0.30 / 0.30 / 0.31 s** |
| 200 held-out hard cases (Python): mean / worst first hit | 0.63 / 1.87 s | **0.52 / 1.00 s** |
| … mean miss distance | 0.83 m | **0.32 m** |
| Least-damped closed-loop pole | ζ = 0.09 | **ζ ≈ 0.46** |

## What v4 changes (details in `overnight/REPORT.md`)
1. **Aiming (`ambush2`):** the aim point is set tp ahead, re-predicted to the same instant on every call, and timed to the laser recharge.
2. **Firing (`fire_best`):** inside the 3σ gate, the laser fires at the target's closest approach.
3. **Damping (`damped` servo):** current feedback adds a virtual resistance that damps the motor's resonant mode. A plain PI-lead outer loop replaces the notch.
4. **Acquisition:** aim after 3 radar samples instead of 6, and slew with a near-time-optimal braking profile.

## Known limitations
- Sensor latency is not modelled in the brief. With 20–50 ms of it, both v3 and v4 need latency compensation (see `COMPARISON_v3_v4.md`).
- Peak motor voltage reaches about 1.8 kV on scenarios A/B/C and about 4.9 kV on the hard cases. The brief sets no limit, so this is stated as an assumption.
- AI use: the v4 changes and the Python analysis were proposed and tested with Claude, and checked against held-out cases. Every claim is logged in `overnight/REPORT.md`.
