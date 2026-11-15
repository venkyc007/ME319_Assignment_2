# Guardian v3 vs v4: which one to submit, and which one is better in the real world

ME319 Assignment 2 · companion to `REPORT.md` · 15 Nov 2026

## TL;DR

| Question | Answer |
|---|---|
| Better on the brief's own conditions? | **v4**, on every Final-Evaluation metric, with no regressions. |
| Better on realistic hardware, both used unchanged? | **Neither is usable.** v3 is slightly less bad (72 % vs 63 % of targets hit). |
| Better on realistic hardware once each gets the "Model R" fixes? | **v4**: 100 % / 2.03 s / 80 % of shots hit, against v3's 100 % / 2.36 s / 70 %. |
| Lower risk to submit? | Now close. The v4 port **was verified in Octave 11.3 on 15 Nov** and reproduces the Python numbers exactly. v3 still has the edge on paperwork: its Bode and margin scripts match its code. |
| **Recommendation** | **Submit v4**, after updating its Bode and margin scripts. If there's no time for those, submit v3.5 (§6), which keeps all of v3's analysis valid. |

What v4 improves is **timing and damping**: when it aims, when it fires, and how well the turret's resonance is damped. It does **not** improve sensing. All three versions (v2→v3→v4) share the same weakness on real hardware: radar noise and latency. The fix for that is the same set of calibration changes the v2/v3 study called Model R, and those changes stack on top of v4 just as well as on v3.

---

## 1. What is different (and what isn't)

Shared and unchanged: the interface, the constant-acceleration EKF (q_jerk = 20²), aiming at the predicted state at tp = 0.5 s, the 3σ covariance fire gate, inverse-plant feedforward, and all assigned parameters.

| # | v3 | v4 | Kind of change |
|---|---|---|---|
| 1 | Aim at t + tp; re-aim on a 0.25 s timer | Aim at t + tp, then **re-predict to that same instant every call**; next aim timed so the target arrives when the laser has recharged | Fire control |
| 2 | Fire the moment the 3σ gate opens (edge of the window) | Inside the gate, **wait one call if the target will be clearly closer** | Fire control |
| 3 | Notch + PI + lead, ω_c = 10 rad/s; the notch *cancels* the ζ = 0.094 electromechanical mode | **Current observer + virtual-resistance minor loop** (Rv = 5 Ω) *damps* the mode to ζ ≈ 0.46; outer loop plain PI-lead, ω_c = 15 rad/s | Servo |
| 4 | Aim after 6 radar samples; linear critically damped slew | Aim after **3** samples; **near-time-optimal (PTOS) braking** slew | Acquisition |

Changes 1–2 are fire-control logic only and leave Gc(s) alone. Changes 3–4 change the servo, so the Bode plots, margins and closed-loop analysis all change with them.

---

## 2. Within the scope of the assignment

### 2.1 Performance (what Final Evaluation scores: success, interception time, accuracy, consistency)

| Test (1 ms simulation, controller every step) | v3 | v4 |
|---|---|---|
| Scenarios A / B / C | 8/8 hits each, first hit 0.44 s, miss 0.6–0.8 m | **10/10 each, first hit 0.30 s, miss 0.30–0.48 m** |
| 200 held-out hard cases (5 manoeuvre families, 0–3 blackouts): targets hit | 100 % | 100 % |
| … mean / worst first hit | 0.63 / 1.87 s | **0.52 / 1.00 s** |
| … pulses per 10 s (max 10) | 7.9 | **9.9** |
| … mean pulse miss distance | 0.83 m | **0.32 m** |
| 100 cases from v3's own R-family (v3 was tuned on R1–100) | 100 % / 0.55 s / 0.83 m | **100 % / 0.42 s / 0.33 m** |
| Plant ±20 % (J, L, Kt) | 100 % / 0.56–0.61 s | **100 % / 0.52–0.53 s** |
| Turret starts at 20°, meas_valid held high | 100 % / 0.62–0.66 s | **100 % / 0.52–0.54 s** |
| Controller called every 5 / 20 / 50 ms | 0.71 / 1.91 / 2.22 s | **0.61 / 1.20 / 1.49 s** |
| Peak motor current / voltage | 164 A / 5.3 kV | **112 A / 4.9 kV** |

**Consistency** is where v4 gains most: the worst case is about half v3's (1.0 s against 1.9 s), and the spread of first-hit times is tighter. The brief explicitly scores consistency.

### 2.2 Analysis content (the 10 paper points)

| Rubric line | v3 | v4 |
|---|---|---|
| Controller design & stability (1.5) | Textbook notch-PI-lead with margins. **Hidden flaw:** the cancelled mode stays at ζ = 0.09, falls to 0.01–0.04 at 20–40 ms call periods, and halves again under ±20 % plant error. A sharp examiner could find it. | Finds the flaw and fixes it the classical way (minor-loop feedback). Damping 0.35–0.54 under plant error. PM 55°, GM 11.9 dB, Ms 1.48. **Stronger, more interesting story.** |
| Estimation & prediction (2) | Same EKF | Same EKF; ambush2 gives a neat sentence: "prediction horizon = time to the next feasible engagement". |
| Originality (2) | Ambush aim point | Ambush plus recharge synchronisation plus damped servo. More original. |
| "Simple, well-motivated" preference | **Simpler** | More moving parts (observer, Rv schedule, slow-call fallback, PTOS). |

### 2.3 Compliance risks: v4 carries more of them

1. **"A conventional LTI compensator Gc(s)".**
   - v4's inner loop is linear and time-invariant: the deadbeat current observer is a fixed linear filter of V and θ̇, and the feedback is −Rv·î.
   - It is still a **two-loop structure, not a single Gc(s)** in unity feedback.
   - The Rv(dt) schedule and the fallback above 30 ms amount to a gain choice made once, at the first call, from the call period. Defensible ("designed once, for the evaluator's sample rate"), but it needs a paragraph.
   - **Paper framing:** "minor-loop (current) feedback modifies the plant to Gp,Rv(s); Gc(s) is then designed for Gp,Rv(s)". This is standard textbook minor-loop design.
2. **tp = 0.5 s.** Both versions place the aim point at t + 0.5 s. Between re-aims, v3 holds the aim point and v4 re-predicts it to the same instant, so in both the *remaining* horizon shrinks below 0.5 s. v4 relies on this more openly. Ask the TA, or frame it as "aim point predicted at tp, refined as new data arrive".
3. ~~Untested port.~~ **Resolved 15 Nov.** `v4_check.m` in GNU Octave 11.3 reproduces the Python prototype exactly: A/B/C 10/10 at 0.30/0.30/0.31 s, R1 0.16 s, R7 0.19 s, same misses, peak V and peak I. v3 in the same Octave gives its known 8/8 at 0.44 s.
   - With the whole simulation stepped at 40 ms, both versions turn erratic: one radar sample lines up with a controller call only every 200 ms. v4 has an unlucky B (first hit 7.8 s against v3's 3.8 s) but is better on A and C. Even v4's code run with all v3 settings switches between 3.8 and 7.8 s on small changes, so this is timing luck, not a v4 defect.
   - If you can, do one confirmation run in real MATLAB.
4. **Figure and analysis scripts** (`make_bode`, `margins_check`, `plant_and_margins`, `make_figures`) in `guardian_v4/` are still the v3 versions and assume the notch. They must be updated before any v4 figures go into the paper.

**Within scope: v4 is the better design. Whether it is the better submission depends on 3 and 4.**

---

## 3. In realistic conditions

Same "worlds" as the v2/v3 study. 60 held-out hard cases (H200–259), Python prototype (`exp_realism.py`, logs `realism_run.log` and `realism_run2.log`).

**Effects:**
- Realistic world: 650 V drive, 0.1° radar noise (5× the brief), 50 ms sensing latency.
- Not modelled here (the earlier study found they barely matter): friction, backlash, encoder quantisation, beam jitter, and a drive current limit.

**"+R" means the same calibration fixes as Model R, applied to either version:**
- Latency-aware prediction (extrapolate by the known latency).
- EKF told the real radar noise.
- Anti-windup and a speed cap of 80 % of what the drive can deliver.
- A probability-of-hit fire gate (P ≥ 0.5) that fires at the peak of P.

### 3.1 Results

| World | v3 unchanged | v4 unchanged | v3 + R | v4 + R |
|---|---|---|---|---|
| Brief | 100 % / 0.61 s / 100 % | **100 % / 0.52 s / 100 %** | 100 % / 0.68 s / 87 % | 100 % / 0.54 s / 99 % |
| 50 ms latency only | 52 % / 2.09 s / 36 % | 50 % / 2.48 s / 27 % | 100 % / 0.72 s / 86 % | **100 % / 0.67 s / 98 %** |
| 650 V drive only | 100 % / 1.40 s / 100 %, **650 A peak** | 100 % / 1.27 s / 100 %, **650 A peak** | 100 % / 1.71 s / 86 %, 82 A | **100 % / 1.70 s / 99 %, 87 A** |
| 0.1° radar only | 100 % / 1.38 s / 70 % | 100 % / 0.98 s / 78 % | 100 % / 0.96 s / 76 % | **100 % / 0.89 s / 83 %** |
| **Realistic (all three)** | 72 % / 3.56 s / 33 % | 63 % / 3.71 s / 26 % | 100 % / 2.36 s / 70 % | **100 % / 2.03 s / 80 %** |
| Brief world, but "+R" tuned for realistic | | | 33 % / 2.29 s / 8 % | 43 % / 2.30 s / 22 % |

Each cell gives targets hit / mean first hit / shots that hit. As a sanity check, v3 + R in the realistic world reproduces the original MATLAB Model R (100 % / 2.32 s / 79 %) closely.

### 3.2 What this shows

- **Unchanged, both fail on real hardware, and v4 fails slightly worse.** Latency is the killer.
  - Both versions aim where the drone *was* 50 ms ago.
  - v4 suffers more because it acts on its estimate more precisely: it fires at the predicted closest approach rather than anywhere inside the gate.
  - The same thing appears at 20 ms latency (REPORT §2): 68 % of v4's shots hit, against 91 % for v3.
- **With the realism fixes, v4 is better in every realistic world.**
  - With latency: 98 % of shots hit, against 86 % for v3 + R.
  - Realistic world: first hit 0.3 s sooner, 80 % of shots hit against 70 %, and 8.8 shots per engagement against 7.2.
  - The v4 improvements (recharge sync, closest-approach firing, damping) still pay off once the sensing problem is handled.
- **The drive limit doesn't stop either version from hitting.** Without anti-windup, though, both ask the drive for 650 A peaks (6.5× a 100 A drive). The earlier study saw this windup as oscillation. My oscillation counter (reversals beyond the target's own) flagged 0 runs everywhere, so I can't confirm the oscillation part here; the current spikes are the measurable symptom.
- **Calibration still cuts both ways.** Either version tuned for realistic hardware but run in the brief's world falls to 33–43 % of targets. As in the v2/v3 study, the realism fixes are a calibration choice, not a better design.
- **Real radar noise forces a different fire gate.** Tell the EKF the true 0.1° noise and the 3σ gate never opens: at 1.5 km, σ ≈ 2.6 m, already more than the 2 m hit radius. Both versions then score 0 % (see `realism_run.log`). Hence the probability-of-hit gate, in both v3 + R and v4 + R.

---

## 4. Good vs bad, summarised

### v4: good
- Faster first hit (−0.11 s mean, about half the worst case), about 10 pulses instead of 8, miss distance about 60 % smaller.
- Fixes a real stability weakness in v3's servo (the cancelled ζ = 0.09 mode). This is the best analysis material in the project.
- Lower peak current (112 A against 164 A).
- More robust to plant mismatch, radar noise and slower controller calls.
- With realism fixes, better than v3 + R in every realistic world tested.

### v4: bad
- Its figure and margin scripts are stale. (The port itself is verified in Octave.)
- More complex: an observer, a dt schedule with a fallback, and the PTOS shaper. The brief prefers simple designs.
- A two-loop servo is harder to present as "a conventional LTI Gc(s)" (it is defensible, but needs care).
- More sensitive to *unmodelled* latency than v3.
- Peak voltage is still about 4.9 kV. Not a brief limit, but it needs stating as an assumption (true of v3 too).

### v3: good
- Verified in MATLAB; analysis and figures consistent with the code.
- Simple, textbook Gc(s); easy to defend in a viva.
- Slightly more tolerant of unmodelled latency.

### v3: bad
- Its notch hides a lightly damped mode that degrades badly at coarse controller rates or with plant error.
- Wastes about 2 pulses per engagement (re-aims before the laser can fire again).
- Fires at the edge of the hit window (0.8 m mean miss).
- Worst case almost twice v4's.

---

## 5. Recommendation

1. ~~Run `guardian_v4/v4_check.m`.~~ **Done in Octave 11.3: exact match.**
2. **Submit v4.** Update `make_bode` and `margins_check` to use L(s) = Gc(s)·Gp,Rv(s), with Gp,Rv = N·Kt / ( s[(J s + b)(L s + R + Rv) + N²·Kt·Ke] ). Present the damped-mode finding as the centrepiece of the controller-analysis section.
3. **If there's no time to redo the Bode and margin work, submit v3.5** (§6), which keeps v3's analysis valid.
4. **In the paper's practical-considerations section,** use the realistic-world table (§3.1). Suggested sentence: *"On today's hardware (650 V, 0.1° radar, 50 ms latency) the unmodified design hits only 63 % of targets; with latency compensation, matched noise, anti-windup and a probability-of-hit gate it hits 100 %, with a first hit at 2.0 s and 80 % of shots on target."*

## 6. Middle option, "v3.5": only the fire-control changes

Keep v3's servo, Gc(s), shaper and n_init = 6, so every v3 figure and margin stays valid. Turn on only `mode = 'ambush2'` and `fire_best = true`.

| 200 held-out hard cases | v3 | v3.5 | v4 |
|---|---|---|---|
| Mean / worst first hit | 0.63 / 1.87 s | 0.61 / 0.94 s | 0.52 / 1.00 s |
| Pulses per 10 s | 7.9 | 9.2 | 9.9 |
| Mean miss | 0.83 m | 0.45 m | 0.32 m |

- v3.5 gets about two-thirds of v4's accuracy gain and halves the worst case, while touching neither the controller nor the paper's analysis.
- It has the same latency sensitivity as v4 (62 % of shots hit at 20 ms unmodelled latency).
- In `guardian_v4/guardian_params.m`, set `servo = 'notch'`, `Rv = 0`, `wc = 10`, `shaper = 'linear'` and `n_init = 6`; keep `mode = 'ambush2'` and `fire_best = true`. The fire-control code is the same code v4_check exercises, so it needs the same MATLAB check.

## Reproduce
- `python exp_round3.py`: in-scope numbers (§2).
- `python exp_realism.py`: realism worlds (§3). It currently runs the "+R2" pass; the unchanged and plain-3σ "+R" rows are in `realism_run.log`.
- `gsim.py` gained one option for this study, `ref_vmax` (the drive-aware speed cap). Its default is off, so all earlier results are unchanged.
