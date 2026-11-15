"""v3 vs v4 in the 'realistic world' of the v2/v3 COMPARISON study (650 V drive,
0.1 deg radar, 50 ms latency), plus the same five-ish 'Model R' fixes applied to each.
Held-out hard cases H200-H259.  Adds an oscillation count (turret reversals beyond
those of the true line of sight, 0.02 rad hysteresis, after acquisition)."""
import math, json, os, sys, time
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_v] = "1"
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gsim, batch

D2R = math.pi / 180
V4 = dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=3,
          servo='damped', Rv=5.0, wc=15.0, Rv_sched=True)
WORLDS = {   # name: (plant/drive overrides, simulate kwargs)
    'brief':      ({}, {}),
    'lat 50ms':   ({}, dict(latency=0.05)),
    'drive 650V': (dict(V_max=650.0), {}),
    'radar 0.1deg': ({}, dict(sig_scale=5.0)),
    'realistic':  (dict(V_max=650.0), dict(latency=0.05, sig_scale=5.0)),
}


def r_fixes(world):
    """Model-R style fixes, calibrated to the world they run in."""
    po, kw = WORLDS[world]
    f = dict(gate='phit', phit_min=0.5, fire_best=True, lat_comp=kw.get('latency', 0.0),
             sig_th=0.02 * D2R * kw.get('sig_scale', 1.0))
    if 'V_max' in po:
        f.update(aw=True, ref_vmax=0.8 * po['V_max'] / (100 * 10.0))
    return f


def reversals(x, h=0.02):
    n, ref, d = 0, x[0], 0
    for v in x:
        if d >= 0 and v < ref - h:
            n += d > 0; d = -1; ref = v
        elif d <= 0 and v > ref + h:
            n += d < 0; d = 1; ref = v
        elif (d >= 0 and v > ref) or (d <= 0 and v < ref):
            ref = v
    return n


def job(a):
    key, ovr, kw = a
    sc = batch.get_case(key)
    R = gsim.simulate(sc, gsim.params(**ovr), log=True, **kw)
    L = R.pop('log')
    acq = np.nonzero(L['miss'] <= 20.0)[0]
    exc = 0
    if len(acq):
        k0 = acq[0]
        exc = reversals(L['th'][k0:]) - reversals(L['th_true'][k0:])
    R['osc'] = exc >= 2
    R['shots'] = R['shots'].tolist()
    return R


def summ(res):
    n = len(res); succ = [r['n_hits'] > 0 for r in res]
    t1 = [r['t1'] for r in res if r['n_hits'] > 0]
    shots = sum(r['n_shots'] for r in res); hits = sum(r['n_hits'] for r in res)
    miss = [s[1] for r in res for s in r['shots']]
    return dict(hit=100 * np.mean(succ), t1=np.mean(t1) if t1 else math.nan,
                t1max=np.max(t1) if t1 else math.nan, shots=shots / n,
                prec=100 * hits / max(shots, 1), miss=np.mean(miss) if miss else math.nan,
                osc=sum(r['osc'] for r in res), Vmax=max(r['maxV'] for r in res),
                Imax=max(r['maxI'] for r in res), n=n)


if __name__ == '__main__':
    cases = ['H%d' % i for i in range(200, 260)]
    pool = batch.pool()
    out = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'realism2.log'), 'a')
    t0 = time.time()
    plan = []
    for w, (po, kw) in WORLDS.items():
        for cand, base in [('v3', {}), ('v4', V4)]:
            plan.append((w, cand + '+R2', dict(base, **po, **r_fixes(w)), kw))
    # mis-calibration: R tuned for the realistic world, run in the brief world
    for cand, base in [('v3', {}), ('v4', V4)]:
        f = r_fixes('realistic'); f.pop('ref_vmax'); f.pop('aw')
        plan.append(('brief', cand + '+R2(told realistic)', dict(base, **f), {}))
    for w, cand, ovr, kw in plan:
        S = summ(pool.map(job, [(c, ovr, kw) for c in cases], chunksize=1))
        S.update(world=w, cand=cand)
        line = ('%-13s %-22s hit=%5.1f%% t1=%5.2f max=%5.2f shots=%4.1f prec=%5.1f%% miss=%5.2f '
                'osc=%2d/%d V=%5.0f I=%4.0f [%4.0fs]' % (w, cand, S['hit'], S['t1'], S['t1max'], S['shots'],
                S['prec'], S['miss'], S['osc'], S['n'], S['Vmax'], S['Imax'], time.time() - t0))
        print(line, flush=True); out.write(line + '\n'); out.flush()
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results.jsonl'), 'a') as fj:
            fj.write(json.dumps(dict(S, round='realism')) + '\n')
