# Guardian turret v2: what we built and how it does

*Written for someone who knows basic control theory (transfer functions, poles, PI/lead, Bode, gain and phase margin) and nothing else about this project.*

## 1. The problem (from the assignment)
- A laser turret has to shoot down a drone flying **800–2000 m away**. The drone moves at up to 150 m/s and weaves with up to 20 m/s² of acceleration.
- We only control the turret's **elevation angle** (up/down). The motor gets a **voltage**, and the turret rotates through a 100:1 gearbox.
- A **radar** reports the drone's distance and angle 20 times a second (every 50 ms). The readings are **noisy**, and the radar can **go blind for up to 0.5 s** (starting any time after t = 1 s).
- The laser fires short **pulses, at most one per second**. A pulse counts as a hit if the beam passes within **2 m** of the drone.
- The brief asks us to point at the target's **future** position, predicted **0.5 s ahead**.
- Three standard tests: **A** (straight flight), **B** (weaving), **C** (weaving plus a 0.5 s radar blackout). The instructor will also use **unseen** test cases.

## 2. What the final version (v2) does
It's a chain of four blocks:

**Radar → (1) estimator → (2) reference → (3) feedback controller → motor**, plus **(4) a fire decision**.

1. **Estimator (Kalman filter).** It smooths the noisy radar and works out the drone's position, velocity and acceleration. It assumes "acceleration changes smoothly", which is a constant-acceleration model. It also keeps track of **how unsure it is**. During a blackout it keeps predicting, and its uncertainty grows.
2. **Reference.** The filter's estimate is turned into the angle the turret should point at, plus that angle's rate and acceleration.
   - **New in v2:** a **reference shaper**, a critically damped 2nd-order filter with an acceleration limit. It turns big jumps in the desired angle into smooth S-curves the turret can actually follow. This mostly matters at the start, when the turret is at 0° and the drone is up at ~13°.
3. **Feedback controller.** We derived the plant from the physics:
   **Gp(s) = 3.125 / [s (s² + 10.5 s + 3130)]**
   That is an integrator (angle from speed) plus a **very lightly damped resonance at 56 rad/s, with ζ ≈ 0.09**. The resonance comes from the motor's inductance interacting with the inertia. The controller has three parts:
   - a **notch** that cancels that resonance (its zeros sit on the plant's bad poles),
   - a **PI** (integral action, so no steady-state error),
   - a small **lead**, sized automatically to give exactly **55° phase margin**.

   On top of that, a **feedforward** term sends the voltage the turret *should* need for the reference's speed and acceleration. The feedback then only has to fix small errors.
4. **Fire decision.** Fire only when the beam is close to the estimated target **and** the filter is confident:
   **(estimated miss) + 3 × (filter's uncertainty) ≤ 2 m**.
   During a blackout the uncertainty grows, so the turret automatically stops firing rather than guessing.

### What changed from v1 to v2, and why

| Change | v1 → v2 | Why, in controls terms |
|---|---|---|
| Crossover frequency ωc | 20 → **10 rad/s** | The loop becomes slower but much more robust. GM 11.4 → **17.3 dB**, peak sensitivity Ms 1.64 → **1.29**. It now stays stable if the plant parameters are off by ±20 % and if the controller only runs every 50 ms. |
| PI zero | 2 → **5 rad/s** | With the zero at 2, the closed loop had a slow pole at −2.2 rad/s, a ~2 s "tail" before the turret settled. Moving it to 5 pushes that pole to about −4.5 rad/s. |
| Reference shaper | none → **on** | The initial voltage spike drops from **4.7 kV to 1.3 kV**, and there's no overshoot toward the 85° end-stop. |
| Radar input handling | only one way of reading the radar → **robust** | A new reading counts only if the valid flag is set **and** the value has changed. v1 broke if the evaluator held "valid = 1" between samples: it kept re-using the old reading as if it were new. |
| Time handling | fragile → **robust** | It ignores small backward time steps, which some MATLAB ODE solvers produce. |

Even though ωc went **down**, the first hit came **earlier**: 0.56 s vs ~1.9 s in v1 on scenarios A/B/C. Speed was never limited by ωc. It was limited by the slow PI pole and the startup transient, and v2 fixed both.

## 3. Our two methods (two ways to read "aim 0.5 s ahead")
The laser is basically instant, so if the turret **continuously** points 0.5 s ahead of the drone, it's always pointing at empty sky and **never hits**. We built two ways around this:

- **Track mode:** the turret continuously follows the drone's **current** estimated position. Feedforward removes tracking lag. The 0.5 s prediction is used to plan the reference and to coast through blackouts. It fires whenever it's lined up and confident.
- **Ambush mode:** predict where the drone will be **0.5 s from now**, swing the turret there, **wait**, and fire as the drone flies into the beam. Then repeat. This uses the 0.5 s literally. The turret is standing still when it fires, so tracking lag doesn't matter.

v2 is set to **track mode by default**. Ambush is still in the code (`P.mode = 'ambush'`).

## 4. Results

**Standard scenarios (v2):**

| | Track | Ambush |
|---|---|---|
| A / B / C hits | 10/10, 10/10, 10/10 | 8/8, 8/8, 8/8 |
| First hit | 0.56 s | 0.44 s |
| Typical miss distance per shot | ~0.4 m | ~0.6–0.8 m |

**Stress test.** 160 deliberately hard targets: sharp jinks, reversals hidden inside blackouts, 1–3 blackouts per run, close crossings, and high elevations. Some evaluator variants (how often the controller is called, how the radar flag behaves, wrong plant parameters) were run on 40 of them. Each cell shows % of targets hit and mean time to first hit:

| Test condition | v1 track | v1 ambush | **v2 (track)** |
|---|---|---|---|
| Hard set, normal 1 ms step | 100 %, 1.96 s | 100 %, 1.26 s | **100 %, 0.68 s** |
| Controller called every 10 ms | 100 %, 1.89 s | 100 %, 1.20 s | **100 %, 0.84 s** |
| Every 20 ms | 90 % (12 unstable runs) | 70 % | **100 %, 0.96 s** |
| Every 50 ms (radar rate) | **0 %** | 38 % | **98 %, 1.21 s** |
| Radar flag held high | 92 % (only 70 % of shots hit) | 100 % (83 % of shots hit) | **100 %, all shots hit** |
| Plant parameters 10–20 % wrong | 68 % (unstable) | 75 % | **100 %, 0.71 s** |

In every v2 row, essentially **every shot fired was a hit** (≥ 98.6 %).

![](guardian_v2/stress_comparison.png)

## 5. What's good
- **It never really misses.** It hit 100 % of the 160 hard targets, and essentially every shot it fires is a hit. That's thanks to the "only fire when confident" rule.
- **It's fast.** First hit is at ~0.6–0.7 s, about 3× faster than v1.
- **It's robust to how the instructor's script might run it.** It copes with different time steps, ways of flagging radar data and integrators, and with a plant that doesn't exactly match the model. v1 failed badly on several of these, and this matters because 5 of the 15 marks come from unseen instructor cases.
- **It handles blackouts sensibly.** It keeps predicting, holds fire while unsure, and resumes once the radar is back.
- **Each design choice has a clear controls reason** (margins, pole locations, notch), which makes the paper easy to justify.

## 6. What's bad / limitations
- **Voltage is still unrealistic at startup.** It peaks at about 1.3 kV and 120 A in the first fraction of a second. The brief gives no limit, so it's "allowed", but a real motor wouldn't take it. If a voltage limit were added, the controller would need **anti-windup** (stopping the integrator from winding up while the output is saturated).
- **The notch relies on knowing the resonance.** It cancels the 56 rad/s mode exactly. If the real plant is off, the cancellation is imperfect. Tests at ±20 % were fine, but that's not a guarantee for bigger errors.
- **Track mode bends the brief.** It doesn't literally keep pointing 0.5 s ahead; it uses the prediction for planning. Ambush follows the wording more literally, but it's less accurate per shot and wasn't stress-tested with the v2 controller. **Ask the TA which reading they intend.**
- **At a 50 ms controller rate there's a small dip**: 98 %, with 2 runs going unstable. That's the toughest case and unlikely in practice.
- **All tests are our own simulations.** The instructor's cases might differ in ways we didn't think of.
- **It's elevation only (a 2D problem), with no gravity on the turret and no friction beyond simple damping.** These are assumptions the brief allows, but they should be stated in the paper.

## 7. Files
- `guardian_v2/proposed/`: the v2 code. It's a drop-in replacement; only `turret_controller.m` and `guardian_params.m` changed from v1.
- `guardian_v2/stress_test/`: the hard-target generator and test harness.
- `guardian_v2/summary.csv`: all the numbers.
- `guardian_v2/stress_comparison.png`: the chart above.
- `guardian_v2/TESTING.md`: technical notes on v2.
- `SUMMARY.pdf`, `RESULTS.md`, `figs/`: the earlier v1 write-up.
