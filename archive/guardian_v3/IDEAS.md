# Ideas & improvements: what we're missing and what to do next

Written 7 Nov 2026 for whoever finishes the project. The deadline is **Tue 10 Nov, 23:59**.
Each idea has: **why** (evidence from our tests), **what** to do, **effort**, and **which rubric marks it helps**.
Rubric: Originality 2 · Estimation & prediction 2 · Modelling 1.5 · Controller & stability 1.5 · Simulation 1.5 · **Final evaluation 5** · Communication & AI 1.5.

## A. Must-do before submission (in this order)
| # | What | Why | Effort | Marks |
|---|---|---|---|---|
| A1 | **Write the paper.** `main.tex` is only an outline. Use `SUMMARY.md`, `V2_EXPLAINED.md`, `guardian_v3/AUDIT.md`, `guardian_v3/REALISM.md` and `guardian_v2/TESTING.md` as source text. | It's 6 of the 15 marks plus a prerequisite for the rest | 6–8 h | all |
| A2 | **Adopt v3 (ambush default)** and state the tp argument: tracking θ(t + tp) with an instantaneous beam can never hit (we measured ~25 m constant miss), so tp is used as an **aim-point horizon**. Report track mode as the comparison. | Only ambush satisfies Sec. 4 ("reference = predicted state at tp") | 0.5 h (+ writing) | Originality, Final eval |
| A3 | **Email/Piazza the TA** with that one-paragraph argument and ask whether ambush is acceptable. | Removes the biggest grading risk | 15 min | Final eval |
| A4 | **Draw the two required figures**: an engineering schematic (turret, trunnion, radar, LOS, predicted point, beam, R_eng circle) and the closed-loop block diagram (radar → EKF → predictor → aim-point → shaper → Gc → Gp, feedforward branch, fire gate). | Explicitly required in Sec. 5.2 | 1–2 h | Communication |
| A5 | **Regenerate every figure and table with v3** (`make_figures`, `make_bode`, `margins_check`) and **update the README** so a clean clone reproduces everything. Push to a **public** repo. | Repos that can't reproduce results "will not be considered" | 1–2 h | Simulation, Communication |
| A6 | **Analytic justification of the two tuned numbers.** q_jerk: tie it to maneuver jerk (e.g. a white-jerk model with σ_j ≈ a_max·ω_maneuver ≈ 20 × 2 = 40 m/s³, giving q ≈ σ_j²·T). fire_k = 3: a one-sided 3σ Gaussian gate gives ≈ 99.9 % confidence that the true miss is within R_eng. | Sec. 4: "analytical … rather than numerical tuning alone" | 1 h | Estimation, Controller |
| A7 | **State the assumptions** in the Introduction: planar, balanced trunnion, no voltage limit (peak ≈ 1.3 kV, 121 A), negligible beam time-of-flight, initial turret angle 0 at rest, ideal gearbox. | Required ("state the assumptions") | 15 min | Modelling |
| A8 | **Run the instructor's evaluation script** the moment it's released, in both modes. Don't retune on it. Report the results with plots. | 5 marks | 1–2 h | Final eval |
| A9 | **AI appendix**: tools (Claude, Octave), what each did, what was accepted/modified/rejected (e.g. lead a = 8 rejected → PM-sized lead; track default rejected → ambush), and how each claim was validated (Monte-Carlo, held-out sets, margin calculations). | Mandatory | 1 h | Communication |

## B. Quick robustness wins (each ≤ 2 h, all keep Gc LTI)
| # | Idea | Evidence | What to do |
|---|---|---|---|
| B1 | **Anti-windup + voltage limit** | With a 400 V limit, 8/20 hard cases oscillated (REALISM §3). Also lets you drop the "no voltage limit" assumption. | Set `P.V_max` (e.g. 400–600 V) and add back-calculation: when the output saturates, feed (u_sat − u) × K_aw back into the PI state. Or stop integrating while saturated. Re-run the stress set. |
| B2 | **Latency compensation** | 50 ms of delay → 70 % success, 100 ms → 40–55 %. This is the #1 real-world killer. | If a latency L is known or assumed, time-stamp measurements and run the EKF update at t − L, then propagate forward by L before forming the reference. Even a simple "extrapolate the estimate by L" recovers most of it. Good originality paragraph. |
| B3 | **Innovation gating (outlier rejection)** | Glint/multipath outliers cut precision to ~98 % and widen misses. | Before each EKF update compute NIS = yᵀS⁻¹y. Skip the update if NIS > χ²₂(0.999) ≈ 13.8. Three lines of code. |
| B4 | **Adaptive measurement noise** | At 5× radar noise the gate is over-confident (precision 73–76 %). Telling the filter the true noise fixes it (REALISM §5). | Estimate R online from the innovation sequence (covariance matching over the last N samples), or inflate R when the mean NIS > 2. |
| B5 | **Probability-of-hit fire gate** | The current gate (miss + 3σ ≤ R) is conservative when σ is small and too loose when the filter is over-confident. | Fire when P(|miss| ≤ R_eng) ≥ 0.99 under N(miss_est, σ⊥²), i.e. Φ((R − m)/σ) − Φ((−R − m)/σ). It's the same idea but a cleaner statistical statement for the paper. |
| B6 | **Best-instant firing** (`cross_fire` option, already coded, off) | In ambush the target crosses the beam. Firing at the predicted closest approach rather than the first moment the gate opens gives smaller misses. | Turn it on and test on the 160 stress cases. Adopt only if the median miss drops and the success rate stays at 100 %. |
| B7 | **Gravity feedforward / integrator pre-load** | Unbalance didn't hurt here, but it's free to handle. | Add τ_g·cos θ/(N Kt) × R_m to V_ff if an unbalance is assumed. |

## C. Bigger ideas (originality, if time allows)
| # | Idea | Why it might matter |
|---|---|---|
| C1 | **IMM filter (CV + CA, or CA with two q levels)** | Maneuvering targets switch between gentle and hard phases. An IMM adapts q automatically and is a classic answer to "maneuver strategy may vary". It adds complexity, and the brief prefers simple, so only add it if it clearly improves the 160-case stress set. |
| C2 | **Hybrid mode: ambush for the first shot, track afterwards** | Ambush gives the fastest first hit and track gives better follow-up accuracy. `mode='hybrid'` is already coded: test on the stress set. Note: track mode doesn't use tp for its reference, so describe the hybrid carefully (the first engagement uses the tp aim-point). |
| C3 | **Optimal aim horizon** | Instead of always aiming exactly tp ahead, check aim points tp ± δ and choose the one with the smallest predicted miss covariance at the crossing time. This keeps tp as the nominal horizon. |
| C4 | **Cascaded control (inner current/velocity loop)** | Industry-standard structure for servo drives. It handles the 56 rad/s electromechanical mode by feedback instead of an exact notch cancellation, so it's robust to plant mismatch, and it makes current limiting natural. It would replace the notch, so weigh it against the requirement for a "conventional LTI compensator" designed from Gp(s). |
| C5 | **Sensitivity study of the notch** | Plot the closed-loop poles as J and L vary ±30 % (`stress_test/mm_scan.m` already exists). A figure for "Controller & stability" showing why ωc was lowered to 10 rad/s. |
| C6 | **Error-budget figure** | Break the miss down into radar noise → estimate σ, prediction error over tp, servo error, beam. Shows understanding (rubric: interpretation). Data for it is in the stress logs. |
| C7 | **Discrete-time analysis** | `dt_scan.m` shows stability vs controller step. Present it as a z-domain plot: the evaluator may call the controller only at 20 Hz. |
| C8 | **Dwell-based engagement (real-world extension)** | Real HELs need seconds on target (REALISM §1). An extension metric, "time the beam stays within 0.5 m", turns the comparison of track vs ambush around: track mode keeps the beam on the target and ambush doesn't. Good discussion point; don't change the graded criterion. |

## D. Things we checked that are fine (don't spend time here)
- Friction, gravity unbalance, backlash, encoder resolution, beam jitter: negligible at the 2 m scale (REALISM §3).
- 10 Hz radar, 2× longer blackouts, 2× harder maneuvers: still 100 % success.
- Evaluator conventions (held valid flag, 10–50 ms steps, Euler vs RK4, plant ±20 %): v3 ambush 100 % in all (AUDIT §5).

## E. Risks to watch
- **The TA says track is intended:** keep track mode ready (`P.mode='track'`) and have the argument for why θ(t+tp) can't be tracked literally.
- **The evaluator calls the controller at a weird rate** or with NaNs: v2/v3 handle pulse/hold/NaN and 1–50 ms steps, but re-test with the real script.
- **The persistent-state reset**: the controller resets at t = 0 or on a big backward time jump. If the evaluator starts at t > 0 for a new case without going backwards, call `clear turret_controller` between cases (mention it in the README).
- **Viva:** every member should be able to explain the plant derivation, the notch-PI-lead design (margins), the EKF, the tp aim-point logic, and the 3σ gate.
