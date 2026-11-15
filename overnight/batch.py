"""Parallel batch runner.  Usage from other scripts:
    from batch import run_set, summarize
    res = run_set(cases, overrides, sim_kwargs)
cases: list of keys ('A', 'R12', 'H7')."""
import math, os, sys, json, time
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_v] = "1"
import numpy as np
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gsim
import hardset  # noqa: patches Scenario.in_bo


_CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'case_cache')


def get_case(key):
    import pickle
    os.makedirs(_CACHE_DIR, exist_ok=True)
    fn = os.path.join(_CACHE_DIR, key + '.pkl')
    if os.path.exists(fn):
        with open(fn, 'rb') as f:
            return pickle.load(f)
    sc = hardset.make_case(int(key[1:])) if key[0] == 'H' else gsim.Scenario(key)
    with open(fn + '.tmp%d' % os.getpid(), 'wb') as f:
        pickle.dump(sc, f)
    os.replace(fn + '.tmp%d' % os.getpid(), fn)
    return sc


def _job(args):
    key, ovr, kw = args
    try:
        P = gsim.params(**ovr)
        sc = get_case(key)
        R = gsim.simulate(sc, P, **kw)
        R['shots'] = R['shots'].tolist()
        R['family'] = getattr(sc, 'family', key[0])
        return R
    except Exception as e:  # keep the batch alive
        return dict(key=key, error=repr(e), n_shots=0, n_hits=0, t1=math.nan)


_POOL = None


def pool():
    global _POOL
    if _POOL is None:
        _POOL = Pool(8)
    return _POOL


def run_set(cases, ovr=None, kw=None):
    ovr = ovr or {}; kw = kw or {}
    return pool().map(_job, [(c, ovr, kw) for c in cases], chunksize=1)


def summarize(res, label=''):
    n = len(res)
    err = [r for r in res if 'error' in r]
    succ = np.array([r['n_hits'] > 0 for r in res])
    t1 = np.array([r['t1'] for r in res], float)
    shots = sum(r['n_shots'] for r in res); hits = sum(r['n_hits'] for r in res)
    misses = np.array([s[1] for r in res for s in r.get('shots', [])])
    hitmiss = np.array([s[1] for r in res for s in r.get('shots', []) if s[2]])
    first_miss = np.array([r.get('miss_first', math.nan) for r in res], float)
    tt = t1[succ]
    S = dict(label=label, n=n, errors=len(err), success=succ.mean() * 100,
             t1_mean=np.nanmean(tt) if len(tt) else math.nan,
             t1_med=np.nanmedian(tt) if len(tt) else math.nan,
             t1_p90=np.nanpercentile(tt, 90) if len(tt) else math.nan,
             t1_max=np.nanmax(tt) if len(tt) else math.nan,
             t1_std=np.nanstd(tt) if len(tt) else math.nan,
             shots_per=shots / n, hits_per=hits / n, precision=100 * hits / max(shots, 1),
             miss_all=np.mean(misses) if len(misses) else math.nan,
             miss_hit_med=np.median(hitmiss) if len(hitmiss) else math.nan,
             miss_first=np.nanmean(first_miss),
             maxV=max((r.get('maxV', 0) for r in res), default=0),
             maxI=max((r.get('maxI', 0) for r in res), default=0))
    if err:
        S['err_example'] = err[0]['error']
    return S


def fmt(S):
    return ("{label:<34s} succ={success:5.1f}% t1 mean={t1_mean:.3f} med={t1_med:.3f} p90={t1_p90:.3f} max={t1_max:.3f} "
            "| shots/case={shots_per:4.1f} prec={precision:5.1f}% miss_all={miss_all:.2f} hitmed={miss_hit_med:.2f} "
            "first={miss_first:.2f} | Vmax={maxV:.0f} Imax={maxI:.0f}{e}").format(
        e=(' ERR=%d %s' % (S['errors'], S.get('err_example', ''))) if S['errors'] else '', **S)


def log_result(S, fname='results.jsonl', extra=None):
    d = dict(S); d['time'] = time.strftime('%H:%M:%S')
    if extra:
        d.update(extra)
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), fname), 'a') as f:
        f.write(json.dumps(d, default=float) + '\n')
