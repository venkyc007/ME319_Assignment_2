# START HERE: ME319 Assignment 2 (Guardian turret), handoff notes

Last updated 7 Nov 2026. **Deadline: Tue 10 Nov 2026, 23:59** (paper PDF + public GitHub repo + peer forms).

## The project in one paragraph
A laser turret must hit a weaving drone 800–2000 m away using a noisy 20 Hz radar that can go blind for up to 0.5 s. Our design: a Kalman filter estimates the drone's position, velocity and acceleration → we predict where it will be **tp = 0.5 s** ahead → the turret points there and waits (**"ambush"**) → the laser fires only when the filter is confident the beam is within 2 m. The turret controller is a notch + PI + lead compensator designed from the motor/gearbox transfer function.

## Which version is current?
| Folder / file | What it is | Use it? |
|---|---|---|
| **`guardian_v3/`** | **Final candidate code** (v2 + compliance fixes) and the three new reports | **Yes: build the submission from this** |
| `guardian_v2/` | v2 code (`proposed/`) and its stress-test tools/results | Reference |
| root `*.m`, `figs/`, `RESULTS.md` | v1 code and results | History only |
| `guardian_v2.zip`, `Claude outputs/` | archives | Ignore |

## Read these, in this order
1. `V2_EXPLAINED.pdf`: plain-English explanation of the design and the two modes (track vs ambush).
2. `guardian_v3/AUDIT.md`: every requirement of the brief checked against the code, plus what v3 changed and why.
3. `guardian_v3/REALISM.md`: how the design would do with real-world sensors and hardware, with sources.
4. `guardian_v3/IDEAS.md`: **the to-do list** (section A = must-do before submission) and improvement ideas.
5. `guardian_v2/TESTING.md`, `RESULTS.md`, `SUMMARY.pdf`: earlier technical notes (v1/v2).

## Results at a glance
| | v1 | v2 (track) | **v3 (ambush, default)** |
|---|---|---|---|
| Scenarios A/B/C | 9/9 hits, first hit ~1.9 s | 10/10, 0.56 s | **8/8, 0.44 s** |
| 160 hard cases: targets hit | 100 % | 100 % | **100 %** |
| 160 hard cases: mean first hit | 1.96 s | 0.68 s | **0.63 s** |
| Controller called every 50 ms | 0 % | 98 % | **100 %** |
| Plant parameters ±20 % | 68 % | 100 % | **100 %** |
| Uses tp = 0.5 s for the reference (brief Sec. 4) | no | no | **yes** |
| Realistic sensors + hardware, all at once (not retuned) | — | 75 % | 74 % |

## Top 5 things still to do (details in `guardian_v3/IDEAS.md`)
1. **Write the paper** (`main.tex` is only an outline): 6 pages IEEE + 1-page AI appendix, schematic and block diagram.
2. Ask the TA whether the ambush reading of tp is acceptable (the argument is in `AUDIT.md`).
3. Regenerate the figures from `guardian_v3/`, update the README, push a **public** repo, check it reproduces from a clean clone.
4. Run the instructor's evaluation script when it's released (don't retune on it).
5. Optional quick wins: anti-windup + voltage limit, latency compensation, outlier gating (IDEAS section B).

## How to run
Octave 8+ (`pkg install -forge control signal` for the analysis scripts) or MATLAB:
```matlab
cd guardian_v3
R = run_scenario('B');          % ambush (default), prints hits / first-hit time
R = runmode('B','track');       % comparison mode
```
