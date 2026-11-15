import sys, time
from batch import run_set, summarize, fmt, log_result

H = ['H%d' % i for i in range(200)]
DAMP = dict(servo='damped', Rv=5.0, wc=15.0, Rv_sched=True)
BASE = dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=2)
CONFIGS = [
    ('C1', dict(BASE, **DAMP)),
    ('C1 phit.99', dict(BASE, gate='phit', phit_min=0.99, **DAMP)),
    ('C1 phit.995', dict(BASE, gate='phit', phit_min=0.995, **DAMP)),
    ('C1 k2.5', dict(BASE, fire_k=2.5, **DAMP)),
    ('C1 nodamp', dict(BASE)),
    ('C1 nodamp phit.99', dict(BASE, gate='phit', phit_min=0.99)),
    ('C1 n3', dict(BASE, n_init=3, **DAMP)),
    ('C1 amax30', dict(BASE, ref_amax=30.0, **DAMP)),
    ('C1 amax12', dict(BASE, ref_amax=12.0, **DAMP)),
]
if __name__ == '__main__':
    t0 = time.time()
    for label, ovr in CONFIGS:
        for setname, cases, kw in [('std', ['A', 'B', 'C'], {}), ('H 1ms', H, {}), ('H 10ms', H, dict(ctrl_dt=0.01))]:
            S = summarize(run_set(cases, ovr, kw), '%s | %s' % (label, setname))
            print(fmt(S), '  [%.0fs]' % (time.time() - t0), flush=True)
            log_result(S, extra=dict(exp='round2b', ovr=str(ovr), set=setname))
