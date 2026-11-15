"""Round 2: combinations, each evaluated at 1 ms, 10 ms, 20 ms, 50 ms controller periods."""
import sys, time
from batch import run_set, summarize, fmt, log_result

H = ['H%d' % i for i in range(200)]
DAMP = dict(servo='damped', Rv=5.0, wc=15.0, Rv_sched=True)
CONFIGS = [
    ('v3', {}),
    ('A2', dict(mode='ambush2')),
    ('A2+fb', dict(mode='ambush2', fire_best=True)),
    ('A2+fb+ptos', dict(mode='ambush2', fire_best=True, shaper='ptos')),
    ('A2+fb+ptos+n2', dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=2)),
    ('A2+fb+damp', dict(mode='ambush2', fire_best=True, **DAMP)),
    ('A2+fb+ptos+damp', dict(mode='ambush2', fire_best=True, shaper='ptos', **DAMP)),
    ('A2+fb+ptos+n2+damp', dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=2, **DAMP)),
]
DTS = [None, 0.01, 0.02, 0.05]

if __name__ == '__main__':
    which = sys.argv[1:] or [c[0] for c in CONFIGS]
    t0 = time.time()
    for label, ovr in CONFIGS:
        if label not in which:
            continue
        for cdt in DTS:
            S = summarize(run_set(H, ovr, dict(ctrl_dt=cdt)), '%s | %s' % (label, '1ms' if cdt is None else '%dms' % (cdt * 1000)))
            print(fmt(S), '  [%.0fs]' % (time.time() - t0), flush=True)
            log_result(S, extra=dict(exp='round2', ovr=str(ovr), ctrl_dt=cdt))
