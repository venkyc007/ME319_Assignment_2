"""Independent 'hard' evaluation set (H-cases), deliberately NOT from the R-case
family used to tune v1-v3.  Five maneuver families, 0-3 blackouts, full
elevation range, maneuvers that may start at t = 0.  All cases are rejection-
sampled inside the brief's envelope (same check_env as the generator)."""
import math
import numpy as np
from gsim import params, Scenario, integrate, smooth_on, check_env, D2R

FAMILIES = ['weave', 'jink', 'turn', 'dive', 'straight']


def _smooth_steps(tg, times, vals, Tr):
    """piecewise-constant vals (n x 2) switching at times, C1 transitions of length Tr"""
    out = np.tile(vals[0], (len(tg), 1)).astype(float)
    for k in range(1, len(times)):
        w = smooth_on(tg, times[k], Tr)[:, None]
        out = out * (1 - w) + vals[k] * w
    return out


def make_case(idx, P=None, family=None):
    P = P or params()
    rng = np.random.default_rng(50000 + idx)
    dt = 1e-3
    tg = np.arange(0, round((P['t_end'] + 1) / dt) + 1) * dt
    fam = family or FAMILIES[idx % len(FAMILIES)]
    for tries in range(2000):
        r0 = rng.uniform(850, 1980)
        el0 = rng.uniform(-7, 82) * D2R
        p0 = r0 * np.array([math.cos(el0), math.sin(el0)])
        if p0[1] <= 5:
            continue
        spd = rng.uniform(90, 150) if rng.random() < 0.65 else rng.uniform(30, 150)
        # heading: mostly inbound, some crossing/outbound
        los = math.atan2(p0[1], p0[0])
        hdg = los + math.pi + rng.uniform(-1.6, 1.6) if rng.random() < 0.8 else rng.uniform(-math.pi, math.pi)
        v0 = spd * np.array([math.cos(hdg), math.sin(hdg)])
        amax = rng.uniform(10, 20)
        t_on = 0.0 if rng.random() < 0.3 else rng.uniform(0.2, 3.0)
        w = smooth_on(tg, t_on, rng.uniform(0.1, 0.6))[:, None]
        if fam == 'weave':
            acc = np.zeros((len(tg), 2))
            for j in range(2):
                for m in range(rng.integers(1, 4)):
                    acc[:, j] += rng.uniform(0.2, 1) * np.sin(rng.uniform(0.2, 3.2) * tg + rng.uniform(0, 2 * np.pi))
        elif fam == 'jink':
            nsw = rng.integers(3, 12)
            times = np.concatenate([[0], np.sort(rng.uniform(0, 10, nsw))])
            ang = rng.uniform(0, 2 * np.pi, nsw + 1)
            mag = rng.uniform(0.4, 1.0, nsw + 1)
            vals = np.column_stack([mag * np.cos(ang), mag * np.sin(ang)])
            acc = _smooth_steps(tg, times, vals, rng.uniform(0.08, 0.4))
        elif fam == 'turn':
            # accel perpendicular to the (initial-velocity-propagated) heading; sign switches
            nsw = rng.integers(1, 6)
            times = np.concatenate([[0], np.sort(rng.uniform(0, 10, nsw))])
            sgn = rng.choice([-1, 1], nsw + 1)[:, None] * np.ones((1, 2))
            s = _smooth_steps(tg, times, sgn, 0.3)[:, 0]
            # integrate heading with turn rate a/v
            hd = np.zeros(len(tg)); hd[0] = hdg
            for k in range(1, len(tg)):
                hd[k] = hd[k - 1] + s[k - 1] * amax * w[k - 1, 0] / spd * dt
            acc = np.column_stack([-np.sin(hd), np.cos(hd)]) * s[:, None]
        elif fam == 'dive':
            nsw = rng.integers(2, 7)
            times = np.concatenate([[0], np.sort(rng.uniform(0, 10, nsw))])
            vy = rng.choice([-1, 1], nsw + 1) * rng.uniform(0.5, 1, nsw + 1)
            vx = rng.uniform(-0.4, 0.4, nsw + 1)
            acc = _smooth_steps(tg, times, np.column_stack([vx, vy]), rng.uniform(0.1, 0.4))
        else:   # straight
            acc = np.zeros((len(tg), 2))
        mx = np.max(np.hypot(acc[:, 0], acc[:, 1]))
        if mx > 0:
            acc = acc / mx * amax
        acc = acc * w
        X = integrate(tg, p0, v0, acc)
        if check_env(X, tg, P):
            break
    else:
        raise RuntimeError('no admissible case %d' % idx)
    # blackouts: 0..3, onset >= 1 s, duration <= 0.5 s
    nb = rng.choice([0, 1, 1, 1, 2, 3])
    bos = []
    t0 = 1.0
    for _ in range(nb):
        on = t0 + (0 if (not bos and rng.random() < 0.25) else rng.uniform(0, 3.0))
        dur = rng.uniform(0.1, 0.5) if rng.random() > 0.3 else 0.5
        if on + dur > P['t_end']:
            break
        bos.append((on, dur)); t0 = on + dur + 0.2
    sc = Scenario('H%d' % idx, P, X=X, noise_seed=70000 + idx)
    sc.bos = bos
    sc.family = fam
    if bos:
        sc.bo_on, sc.bo_dur = bos[0]
    return sc


def in_bo_multi(sc, tt):
    for on, d in getattr(sc, 'bos', []):
        if on - 1e-9 <= tt < on + d - 1e-9:
            return True
    return False


# monkey-patch Scenario.in_bo to honour multiple blackouts when present
_orig = Scenario.in_bo


def _in_bo(self, tt):
    if hasattr(self, 'bos'):
        return in_bo_multi(self, tt)
    return _orig(self, tt)


Scenario.in_bo = _in_bo
