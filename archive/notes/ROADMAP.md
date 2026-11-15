# ME319 Assignment 2 — Guardian turret: roadmap

**Deadline:** Tue 10 Nov 2026, 23:59 (no extensions). Today: Thu 5 Nov. ~5 days.
**Deliverables:** (1) ≤6-page IEEE two-column paper + 1-page AI appendix (PDF, uploaded by team rep), (2) **public** GitHub repo with `turret_controller.m`, `turret_plant.m`, scripts, README that reproduces every figure, (3) confidential peer-evaluation form from **every** member.

## Where the marks are (15)
| Criterion | Pts | What wins it |
|---|---|---|
| **Final evaluation (instructor cases)** | **5.0** | Robust hits on *unseen* maneuvers + blackouts → test on many random cases, never tune per case |
| Originality & concept | 2.0 | One clear, justified idea beyond textbook (see "Design ideas") |
| Estimation & prediction | 2.0 | Justified filter model, noise/blackout handling, parameter choice |
| Mathematical modelling | 1.5 | Clean derivation of Gp(s), estimator, predictor, closed loop |
| Controller & stability | 1.5 | Bode/root locus/margins, steady-state error analysis, link to bandwidth |
| Simulation & results | 1.5 | Plots + quantitative metrics A/B/C + Monte-Carlo |
| Communication & AI appendix | 1.5 | Tidy IEEE format, labelled schematic + block diagram, honest AI log |

## Key numbers already derived
- Reflected inertia J_eq = 1200 + 100²·0.2 = **3200 kg m²** (motor rotor is 62 % of it).
- **Gp(s) = 1000 / (320 s³ + 3350 s² + 1.0015×10⁶ s) = 3.125 / [s (s² + 10.47 s + 3129.7)]**
- Poles: 0, −5.23 ± 55.70j → **ωn = 55.9 rad/s, ζ = 0.094** (lightly damped electromechanical mode from back-EMF coupling). This is the main controller-design challenge.
- Low-frequency behaviour ≈ velocity servo: ω_turret ≈ V/1000 → tracking 0.19 rad/s needs ~190 V. Brief gives **no voltage limit** — state the assumption.
- P-only control: gain ceiling ≈ 10 470 V/rad → crossover can't exceed ~10 rad/s. A notch (complex zeros on the resonant pair) + PI + lead reaches ωc = 20 rad/s. **Fixed 6 Nov:** lead now sized from PM* = 55° → a = 1.02, GM 11.4 dB, Ms 1.64 (was PM 105.5°).
- Accuracy budget: R_eng = 2 m ⇒ **1.0 mrad at 2000 m, 2.5 mrad at 800 m**. Radar σθ = 0.35 mrad (0.7 m at 2 km) → filtering essential. Beam is a ray, so **only the perpendicular (angular) error matters; range error is irrelevant to a hit.**
- Max LOS rate ≈ v/r ≈ 0.19 rad/s; max LOS accel ≈ 0.06 rad/s² → type-1 loop alone gives ~mrad-level lag ⇒ need feedforward or type-2.

## ⚠ Decision needed from you: what tp = 0.5 s means
Laser time-of-flight is negligible, so a turret that *continuously* points at the target's position 0.5 s ahead leads it by up to v·tp ≈ 75 m and **never hits**. Consistent readings:
1. **Aim-point ("ambush") fire control** — slew to the elevation predicted at t+tp, hold, fire as the target crosses the beam (fire timing uses the fresh estimate). Uses tp literally; turret is stationary at fire time, so tracking lag doesn't matter. Implemented as `P.mode='ambush'`.
2. **Lag-matched tracking** — feed θ_pred(t+tp) as reference and shape the loop so it behaves like a 0.5 s delay at low frequency (Kv = 1/tp). Elegant on paper, weak against maneuvers.
3. **Continuous track of current estimate + inverse-plant feedforward**, tp used for the predicted reference/derivatives & blackout coasting. Implemented as `P.mode='track'` (best hit rate expected, loosest reading of the brief).

My recommendation: lead with (1) as the headline design, report (3) as comparison; **ask the TA/Prof which reading is intended** (one email/Piazza post). This choice affects the 5-pt final-eval score.

## Design ideas for the originality marks
- **Covariance-gated firing** (implemented): fire only when estimated perpendicular miss + k·σ⊥ ≤ R_eng, σ⊥ from the EKF covariance projected on the beam normal. Directly trades interception time vs accuracy; k is the one knob, justified statistically.
- **Notch-PI-lead compensator** that cancels the ζ=0.094 mode, with a sensitivity study (±20 % J, Lm) to show robustness of the cancellation.
- Constant-acceleration EKF in Cartesian with q chosen from the a_max/maneuver bandwidth; blackout handled by open-loop CA coast with growing covariance (gate automatically suppresses firing).
- Optional stretch: IMM (CV + CA) — only if time permits; brief prefers simple.

## Plan & time estimate
| # | Task | Effort | Who |
|---|---|---|---|
| 1 | Plant derivation, controller design + margins | 2 h | Claude (done: numbers), you review |
| 2 | Code: generator, plant, EKF/predictor, controller, harness | 4 h | Claude — **done, runs clean (6 Nov)** |
| 3 | First run in MATLAB/Octave, debug | 1–2 h | **Done in Octave 8.4 (6 Nov)**; MATLAB check still yours |
| 4 | Decide tp interpretation (TA) | 15 min | **You** |
| 5 | Tune q, fire_k, wc on Monte-Carlo set R1…R100 (never on A/B/C individually) | 3 h | **Done (6 Nov)**: q=20², wc=20, fire_k=3; PM fixed to 55° — see RESULTS.md |
| 6 | Figures + metrics tables | 2 h | Claude |
| 7 | Paper draft (LaTeX IEEEtran) + block diagram/schematic | 5 h | Claude drafts, you edit |
| 8 | Instructor eval script when released → run, report | 1–2 h | **You** obtain, Claude/you run |
| 9 | GitHub repo public, README, reproduce-from-clean check | 1 h | **You** create/push |
| 10 | AI appendix, final read, Overleaf compile, submit + peer form | 2 h | You (Claude drafts appendix) |

Total ≈ 22–25 h of work; your hands-on share ≈ 6–8 h (review, decisions, MATLAB runs, repo, submission). Suggested schedule: Thu–Fri code runs + tuning; Sat figures; Sun–Mon paper; Tue eval cases + submit (buffer).

## What only you can do
- Team decision (solo vs up to 5) — score is shared and divided by peer factors.
- Ask the TA about tp; get the evaluation script/cases when released.
- Create the **public** GitHub repo and push (I can't push from here).
- Run in MATLAB to confirm compatibility (I target Octave 8 + MATLAB, no toolbox in the runtime files).
- Understand every equation — a viva is possible.
- Submit the paper form and your own peer-evaluation form.
