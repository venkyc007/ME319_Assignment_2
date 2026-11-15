# Guardian turret — what we did, in plain English

**The project in one line:** a turret has to point a laser at a flying target (800–2000 m away, weaving around) using only a noisy radar, and fire when the beam will actually pass within 2 m of it. This is ME319 Assignment 2.

## What the code does
1. **Guess where the target really is.** The radar is jittery and sometimes cuts out completely. A filter (an EKF, a "smart averaging" tool) cleans up the readings, estimates the target's speed and acceleration, and keeps guessing during radar blackouts.
2. **Move the turret there.** A controller sends voltage to the motor. The motor drive has a nasty built-in wobble at about 56 rad/s, so the controller includes a "notch" that cancels it.
3. **Only fire when confident.** It fires when the beam is close to the target *and* the filter is confident about where the target is. One knob, `fire_k`, sets how sure it has to be. If the radar cuts out, the filter gets unsure and the turret simply stops firing until it's sure again.

It works in two styles:

- **Track mode:** follow the target continuously and fire when lined up.
- **Ambush mode:** point where the target will be 0.5 s from now, wait for it to fly into the beam, then fire.

## What was done this session
- **Got it running.** The code had never been run before. It worked first try in Octave (free MATLAB). I also made it 5× faster so I could test lots of cases.
- **Fixed the controller.** It was way too "cautious": phase margin 105°, where about 55° is normal. That also made it demand huge voltages. It's now tuned to 55° and still comfortably stable (gain margin 11 dB).
- **Tuned 3 settings fairly.** I tuned the filter's "how twitchy is the target" setting, the controller speed and the firing confidence on **100 random target paths**, not on the three standard test cases. That avoids just memorising the test.
- **Double-checked on 100 *new* random paths** that weren't used for tuning. The results were the same, so it isn't overfitting.

## Results
| | Track mode | Ambush mode |
|---|---|---|
| Targets hit (out of 100 random + 100 new) | **all of them** | **all of them** |
| Shots that hit | 100 % | 100 % |
| Time to first hit (typical) | ~2.0 s | ~1.0 s |
| How close the beam passes (typical) | ~0.3 m | ~0.9 m |
| Standard cases A / B / C | 9/9, 9/9, 9/9 hits | 8/8, 8/8, 7/7 hits |

(A hit means within 2 m. Scenario C includes a radar blackout, and the turret correctly holds fire during it.)

## Good stuff
- Never missed a target in 200 random tests, and never wasted a shot.
- Handles radar blackouts sensibly: it stops firing instead of guessing.
- Settings were picked by fair testing, not by fiddling with the official cases.
- **Ambush** gets the first hit about 1 s sooner. **Track** is about 3× more accurate per shot.

## Not-so-good stuff / honest caveats
- **Unrealistic voltage at the start.** In the first 0.3 s the turret swings up from level, and the motor briefly gets ~4,700 V. The assignment doesn't set a limit, but a real motor wouldn't take that. Adding a limit needs an extra fix (anti-windup), or accuracy drops a lot.
- **Track mode is a bit slow to settle** (~2 s to first hit). One part of the controller (the "PI zero") is the bottleneck, not the speed setting. Changing it is the next thing to try.
- **The random tests may be too easy.** Both modes scored 100 %, so the tests can't tell which design is truly better. The instructor's cases might be harder.
- **Which mode is "correct" is still an open question.** The brief's 0.5 s prediction wording can be read either way, so we need to ask the TA.

## What's in this folder
| File / folder | What it is |
|---|---|
| `SUMMARY.md` / `SUMMARY.pdf` | This page |
| `RESULTS.md` | Full technical details: all tuning tables, numbers and method |
| `figs/scen_A_track.png` … `scen_C_ambush.png` | One figure per test case. Top: where the turret points vs the target. Middle right: how close each shot was (green dot = hit). |
| `figs/montecarlo.png` | Left: effect of the firing-confidence setting. Right: how fast targets get hit across all random tests. |
| `figs/bode_loop.png` | Controller before vs after the fix (a frequency plot) |
| `*.m` files | The MATLAB/Octave code. Run `runmode('A','track')` to try one case. |
| `ROADMAP.md`, `README.md` | Project plan and code guide |
