# Guardian — first-run, compensator fix and Monte-Carlo tuning (6 Nov 2026)

Octave 8.4.0 (+ control, signal). All numbers reproducible with the scripts listed at the end.

## 1. Debug status
All six runs (`A`/`B`/`C` × `track`/`ambush`) completed on the first attempt without errors or warnings.
Changes made:

| File | Change | Why |
|---|---|---|
| `target_motion_generator.m` | `interp1` → direct linear indexing on the 1 ms truth grid (`grid_interp`) | 5× speed-up (38 s → 7.4 s per run); bit-identical results |
| `design_controller.m` | Lead ratio now **derived from a target phase margin** (`PM_target`) instead of fixed `lead_a = 8` | Fixes PM = 105.5° (see §2) |
| `guardian_params.m` | `PM_target = 55`, `lead_a = []` (override only), tuned `q_jerk = 20²`, `fire_k = 3` | §2–3 |
| `turret_controller.m`, `run_scenario.m` | Extra debug/log fields (`converged`, `in_front`, `true_miss`) | Enable offline replay of the fire gate |
| new | `runmode.m`, `sweep_batch.m`, `launch.sh`, `replay_fire.m`, `analyse_sweep.m`, `margins_check.m`, `make_figures.m`, `make_mc_figure.m`, `make_bode.m`, `figvis.m` | Harness, tuning, figures |

## 2. Phase-margin fix
With the notch in place, the rest of the loop (notch-cancelled plant + PI, z_i = 2) already has
∠G_rest(j20) = −125.6°, so it has ≈ 54° PM *before* any lead. The fixed a = 8 lead added another +51° → PM 105.5°.
It also multiplied the compensator's high-frequency gain (K·a ≈ 57 kV/rad), which caused the 13 kV spike on the initial 0.23 rad acquisition step.

New rule (analytic, for the paper): φ_m = PM* − (180° + ∠G_rest(jω_c)), a = (1 + sin φ_m)/(1 − sin φ_m), a = 1 if φ_m ≤ 0.

| Design | K [V/rad] | a | GM | PM | ω_c | M_s | HF gain [V/rad] |
|---|---|---|---|---|---|---|---|
| old (a = 8) | 7 086 | 8.00 | 12.1 dB @ 81 | **105.5°** | 20 | 1.57 | 56 688 |
| **new (PM* = 55°)** | 19 847 | 1.02 | 11.4 dB @ 55 | **55.0°** | 20 | 1.64 | 20 239 |
| ω_c = 15 | 14 908 | 1 | 14.0 dB | 60.4° | 15 | 1.45 | 14 908 |
| ω_c = 30 | 23 673 | 1.72 | 8.9 dB | 55.0° | 30 | 1.93 | 40 666 |

In every variant the slowest closed-loop pole is ≈ −2.2 rad/s, set by the PI zero (z_i = 2), not by ω_c. Its tail
(~1.8 s) dominates the track-mode first-hit time (see §3).

## 3. Tuning on R1–R100 (never on A/B/C)
Method: one closed-loop simulation per (mode, q_jerk, ω_c). In **track** mode, firing does not affect the turret trajectory, so
`fire_k` was swept by replaying the gate offline on the logged estimates. The replay was checked against a live run and matches it exactly.
In **ambush** mode firing triggers a re-aim, so each `fire_k` was a full simulation.

**q_jerk** (track, ω_c = 20, fire_k = 2):

| q_jerk | success | mean t₁ [s] | precision | median miss | RMS ⊥ miss (t>2 s) | est. RMS |
|---|---|---|---|---|---|---|
| 10² | 100 % | 1.86 | 99.9 % | 0.28 m | 0.45 m | 0.70 m |
| **20²** | 100 % | 1.87 | 99.9 % | 0.29 m | 0.48 m | 0.75 m |
| 40² (old) | 100 % | 1.88 | 99.8 % | 0.33 m | 0.55 m | 0.83 m |
| 80² | 100 % | 1.84 | 99.9 % | 0.38 m | 0.64 m | 0.94 m |

Choice: **20²**. Accuracy is within 0.03 m of the best, and it is not at the edge of the tested range, which leaves margin for sharper maneuvers in the instructor cases.

**ω_c** (track, q = 20², fire_k = 2): 15 → t₁ 1.98 s; **20 → 1.87 s**; 30 → 1.82 s.
Choice: **20**. Going to 30 buys only 0.05 s and costs 2.5 dB of GM and raises M_s to 1.93.

**fire_k**:

| fire_k | track: precision / mean t₁ | ambush: precision / mean t₁ / median miss |
|---|---|---|
| 1 | 99.4 % / 1.79 s | 88.6 % / 1.34 s / 1.57 m |
| 2 | 99.9 % / 1.87 s | 98.8 % / 1.21 s / 1.22 m |
| **3** | **100 % / 2.00 s** | **100 % / 1.21 s / 0.87 m** |
| 4 | 100 % / 2.12 s | — |

Choice: **3** (a 3σ gate). In ambush mode it is free; in track mode it costs ≈ 0.13 s.

## 4. Final design — results
Parameters: q_jerk = 20², ω_c = 20 rad/s, PM* = 55° (a = 1.02), z_i = 2, ζ_notch = 0.7, fire_k = 3, n_init = 6, V_max = ∞.

**Monte-Carlo interception (case success = ≥ 1 hit; precision = hits / shots)**

| Set | Mode | Success | t₁ mean / median / p90 [s] | Precision | Shots/case | Median miss |
|---|---|---|---|---|---|---|
| R1–R100 (tuning) | track | **100 %** | 2.00 / 2.05 / 2.38 | 100 % | 8.4 | 0.29 m |
| R1–R100 (tuning) | ambush | **100 %** | 1.21 / 0.98 / 1.71 | 100 % | 7.5 | 0.87 m |
| R101–R200 (held out) | track | **100 %** | 1.98 / 2.00 / 2.30 | 100 % | 8.4 | 0.32 m |
| R101–R200 (held out) | ambush | **100 %** | 1.19 / 0.98 / 1.65 | 100 % | 7.6 | 0.85 m |

The held-out results match the tuning set, so there is no sign of over-fitting.

**Standardized scenarios**

| Scenario | Mode | Hits/shots | First hit | Mean miss | RMS ⊥ miss (t>2 s) | Est. RMS |
|---|---|---|---|---|---|---|
| A | track | 9/9 | 1.65 s | 0.34 m | 0.42 m | 0.76 m |
| B | track | 9/9 | 1.85 s | 0.46 m | 0.50 m | 0.71 m |
| C | track | 9/9 | 1.85 s | 0.39 m | 0.41 m | 0.75 m |
| A | ambush | 8/8 | 0.95 s | 1.13 m | (aim-ahead by design) | 0.76 m |
| B | ambush | 8/8 | 0.98 s | 1.00 m | — | 0.71 m |
| C | ambush | 7/7 | 1.55 s | 0.84 m | — | 0.75 m |

In C, the gate holds fire during the 1.0–1.5 s blackout as σ⊥ grows (figure panel "Estimator"). The first ambush shot therefore moves from 0.98 s to 1.55 s, with no wasted pulse.

## 5. Open issues / caveats (for you to decide)
1. **Voltage.** Peak |V| is 4.7 kV (≈ 400 A) during the first ≈ 0.3 s acquisition slew from θ = 0 to ≈ 13°. After 0.3 s, |V| ≤ 86 V in track mode, but ambush re-aims reach 500–750 V. The brief gives no limit.
   A quick test with saturation (scenario B, track): 600 V clamp → still 9/9; 300 V clamp → 8/8, but RMS ⊥ miss rises to 8.5 m.
   The cause is **integrator wind-up**: the compensator has no anti-windup. If you adopt a voltage limit, add anti-windup (or a rate-limited acquisition reference) before re-tuning.
2. **First-hit time is limited by z_i, not ω_c.** The PI zero at 2 rad/s leaves a slow closed-loop pole at about −2.2 rad/s. The feedforward already carries the velocity, so a higher z_i (or a 2-DOF prefilter) should cut track-mode t₁. It was not in the requested tuning set and is untested.
3. **The R cases are easier than they may look.** Both modes hit 100 %, so the success metric no longer separates designs. Only interception time and accuracy do. If the instructor cases are harsher (faster maneuvers, longer or earlier blackouts at the 0.5 s limit), re-check with a harder generator.
4. **The tp interpretation (track vs ambush) is still your call.** Ambush intercepts ≈ 0.8 s sooner, and track is ≈ 3× more accurate per shot.
5. The notch-cancelled plant pair (−5.23 ± 55.7j, ζ = 0.094) is still a closed-loop pole pair. It is hidden from the reference but not from disturbances or model error; a ±20 % J/L sensitivity study is still to do.

## Reproduce
```
octave-cli --eval "R = runmode('B','ambush');"                 % one scenario
./launch.sh myrun track "struct()" 1 100                         % Monte-Carlo (2 workers)
octave-cli --eval "analyse_sweep('myrun',[1 2 3]);"              % replay fire_k in track mode
octave-cli --eval "margins_check();"                             % GM/PM/Ms (needs control pkg)
xvfb-run -a octave-cli --eval "make_figures(); make_mc_figure; make_bode"   % figures/
```
