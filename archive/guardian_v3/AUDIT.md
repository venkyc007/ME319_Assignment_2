# v2 compliance audit against the assignment ("assigned parameters")

Audit date: 7 Nov 2026. Brief: *ME319 Assignment 2, Autumn 2026*. Code audited: `guardian_v2/proposed/`.
Every requirement in the brief is listed below with where the code handles it and a verdict.
Two problems were found, and both are fixed in `guardian_v3/` (see the end of this document).

Legend: ✅ satisfied · ⚠️ satisfied but must be stated/justified in the paper · ❌ not satisfied in v2 (fixed in v3)

## 1. Physical plant (brief Sec. 2)
| Assigned parameter | Value in brief | In code | Verdict |
|---|---|---|---|
| Turret + cradle inertia J_turret | 1200 kg m² | `guardian_params.J_turret = 1200` | ✅ |
| Motor rotor inertia Jm | 0.20 kg m² | `Jm = 0.20`, reflected as N²·Jm → J_eq = 3200 | ✅ |
| Gear ratio | 100:1 (motor = 100 × turret speed) | `N = 100`; torque N·Kt·i, back-EMF Ke·N·θ̇ | ✅ |
| Turret-side viscous damping b | 1500 N m s/rad | `b = 1500`, acts on turret speed | ✅ |
| Kt, Ke, Rm, Lm | 10 N m/A, 10 V s/rad, 1.0 Ω, 0.10 H | same | ✅ |
| Elevation limits | −10° … +85° | `theta_min/max`; end-stops in `turret_plant.m`; reference clamped 1° inside; reference shaper prevents overshoot | ✅ |
| Plant "derived from first principles" | — | `turret_plant.m` (J_eq θ̈ = N Kt i − b θ̇; L di/dt = V − R i − Ke N θ̇) | ✅ |
| Gp(s) used for controller design | linearised small-perturbation model | `design_controller.m` builds Gp(s) = 3.125 / [s (s² + 10.47 s + 3129.7)] from the same parameters | ✅ |
| Gravity / friction / voltage limit | not specified | Neglected; no voltage limit (peak ≈ 1.3 kV at start-up) | ⚠️ state as assumptions |

## 2. Sensor and engagement (brief Sec. 2)
| Assigned parameter | Value | In code | Verdict |
|---|---|---|---|
| Radar rate | 20 Hz (50 ms) | `Ts_radar = 0.05`; generator samples on the 50 ms grid | ✅ |
| Range noise σr | 1 m | `sig_r = 1.0`; used in the EKF R matrix | ✅ |
| Angle noise σθ | 0.02° | `sig_th = 0.02*pi/180`; used in the EKF R matrix | ✅ |
| Range-rate / velocity not measured | — | EKF estimates them from (r, θ) only | ✅ |
| Blackout duration t_bo | ≤ 0.50 s | generator: C = 0.5 s; R cases ≤ 0.5 s | ✅ (v3 also checks it) |
| Blackout onset t_boo | ≥ 1.00 s | C at 1.0 s; R cases 1–7 s | ✅ (v3 also checks it) |
| During blackout | use only information from before the loss | EKF predicts open-loop, covariance grows, fire gate closes | ✅ |
| Laser time of flight | negligible | beam is an instantaneous ray | ✅ |
| Pulse firing on fire_cmd 0→1; holding 1 gives no new pulse | — | controller outputs a one-call pulse | ✅ |
| Recharge 1.0 s; early 0→1 transitions ignored | — | controller waits ≥ 1.0 s + 1 µs; harness ignores early pulses | ✅ |
| Hit = min. distance beam-ray ↔ target ≤ R_eng = 2 m | — | `ray_miss` (ray, not infinite line) | ✅ |
| t_end = 10 s; results cover 0 ≤ t ≤ t_end | — | all plots 0–10 s | ✅ |

## 3. Target envelope and scenarios (brief Sec. 3)
| Requirement | In code | Verdict |
|---|---|---|
| 800 ≤ r ≤ 2000 m, v ≤ 150 m/s, a ≤ 20 m/s² | A: r 832–1953 m, v 120, a 0. B/C: r 942–1953 m, v ≤ 121, a ≤ 16.6. R and stress cases: rejection-sampled inside the envelope | ✅ |
| "The trajectory generator **shall enforce** the limits for every generated trajectory" | v2: R cases enforced, but A/B/C only issue a **warning** | ❌ → v3 raises an **error** for any scenario outside the envelope or the blackout bounds |
| A: non-maneuvering; B: smooth x- and y-maneuver from t = 1 s; C: B + 0.5 s loss at 1.0 s | as specified; maneuver ramps in smoothly (continuous acceleration) | ✅ |

## 4. Required function interfaces (brief Sec. 3)
| Required | v2 | Verdict |
|---|---|---|
| `[r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)` | same (optional 4th output = truth) | ✅ |
| `[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)` | same (optional 3rd output = debug) | ✅ |
| `[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)` | same | ✅ |
| `meas_valid` = a *new* measurement is available | handled whether the evaluator pulses it or holds it high | ✅ |
| Controller depends only on its inputs (no peeking at truth) | yes: uses only its arguments and `guardian_params` / `design_controller` | ✅ (these two files must ship with it) |

## 5. Controller design requirements (brief Sec. 4)
| Requirement | v2 | Verdict |
|---|---|---|
| Architecture: radar → state estimation → prediction → **predicted elevation reference** → feedback controller → turret | estimation ✅, prediction ✅, controller ✅ | see next row |
| "predict the target's future state at the prescribed prediction horizon **tp = 0.50 s**"; this predicted state determines the **future elevation reference**, and "the controller shall track the predicted elevation reference" | **Ambush mode:** reference = elevation predicted at t + tp ✅. **Track mode (v2 default):** reference = **current** estimate (`tau_ref = 0`); the tp prediction is computed but only logged | ❌ for the v2 default → v3 makes ambush the default. Track stays available as a comparison. |
| Effects of noise, prediction error and elevation limits considered | covariance-gated firing; reference clamp + shaper | ✅ |
| Gc(s) is a conventional **LTI** compensator | notch × PI × lead, ZOH-discretised | ✅ |
| Supported analytically, not just tuned | ωc, z_i, PM* from margins and pole locations ✅; q_jerk and fire_k chosen by Monte-Carlo | ⚠️ add analytic justification: q from maneuver jerk, k = 3 as a 3σ gate |
| Extra blocks around Gc | inverse-plant feedforward (LTI) + reference shaper (critically damped 2nd order with an **acceleration limit**, i.e. a nonlinear element in the reference path only) | ⚠️ say explicitly that the loop around the plant is LTI and the saturation only shapes the reference |
| One design for all scenarios, no per-case retuning | tuned on random/stress sets only | ✅ |

### Why track mode can't simply use tp
If the turret tracked θ(t + 0.5 s) perfectly, the laser (time of flight ≈ 0) would always point where the target *will be*, never where it *is*. We tested this in v2 track mode with `tau_ref = tp`:

| Scenario | Hits / shots | Typical perpendicular miss after 2 s |
|---|---|---|
| A | 1/1 (a lucky hit while sweeping up at start-up) | 24 m |
| B | 1/1 | 25 m |
| C | 1/1 | 25 m |

So the literal "track θ(t + tp)" reading cannot work. The brief's wording is only consistent with an **aim-point (ambush)** interpretation: predict where the target will be in tp, put the beam there, and fire as the target arrives. This is a strong point to make explicitly in the paper. Ask the TA to confirm, but ambush is the defensible default either way.

### Evidence that ambush is a safe default (v2 controller, same 160 hard cases as `guardian_v2/summary.csv`)
| Test | Track | **Ambush** |
|---|---|---|
| 160 hard cases, 1 ms step | 100 % hit, t₁ 0.68 s, precision 100 % | **100 %, t₁ 0.63 s, precision 99.9 %**, hit miss 0.87 m |
| 20 ms controller step (40 cases) | 100 %, t₁ 0.96 s | **100 %, t₁ 1.15 s** |
| 50 ms controller step (40 cases) | 97.5 % (1 failure) | **100 %, t₁ 1.86 s** |
| Radar flag held high (40 cases) | 100 %, t₁ 0.68 s | **100 %, t₁ 0.67 s** |
| Plant +20 % J, L, −20 % Kt (40 cases) | 100 %, t₁ 0.62 s | **100 %, t₁ 0.61 s** |
| Plant −20 % J, L, +20 % Kt (40 cases) | 100 %, t₁ 0.50 s | **100 %, t₁ 0.49 s** |
| Standard scenarios A/B/C | 10/10 each, t₁ 0.56 s, miss ≈ 0.4 m | **8/8 each, t₁ 0.44 s**, miss ≈ 0.6–0.8 m |

Ambush intercepts at least as early, is never worse on success, and survives the 50 ms case that track fails. Its trade-off: each hit is less centred (~0.85 m vs ~0.45 m), and it fires fewer shots (~8 vs ~10 per run).

## 6. Deliverables (brief Secs. 5, 7, 9): status on 7 Nov
| Item | Status |
|---|---|
| ≤ 6-page IEEE paper with the required sections | ❌ `main.tex` is still a section outline with no text. **Highest priority (due 10 Nov).** |
| Labeled engineering schematic + closed-loop block diagram | ❌ not drawn yet |
| Plots of target motion, estimates, predictions, turret response, tracking error | ✅ `figs/` (v1) — regenerate them for the final design (`make_figures`) |
| Final evaluation with the instructor's script/cases | ⏳ not released yet; the code is ready for several evaluator conventions |
| Public GitHub repo: code, parameters, scripts, README that reproduces every figure | ⚠️ the README still describes v1; update it for v3 and test from a clean clone |
| 1-page AI appendix (tools, contributions, accepted/modified/rejected, validation) | ❌ to write. Sources: this folder's reports (`RESULTS.md`, `TESTING.md`, `AUDIT.md`, `REALISM.md`) |
| Peer-evaluation form from every member | ⏳ each member must submit it |

## 7. What changed in v3 (`guardian_v3/`)
v3 is a drop-in copy of `guardian_v2/proposed/` with only these edits:
1. `guardian_params.m`: `P.mode = 'ambush'` (was `'track'`), with comments explaining why.
2. `target_motion_generator.m`: the envelope and blackout bounds are now **enforced** (error instead of warning) for every scenario.

v3 results: A, B, C each 8/8 hits, first hit 0.44 s. R1 and R7 8/8, first hit 0.33–0.34 s.
To compare with track mode: `runmode('B','track')`.
