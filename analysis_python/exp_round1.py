"""Round 1: one-at-a-time changes vs v3 on H200 (1 ms) and A/B/C."""
import sys, time
from batch import run_set, summarize, fmt, log_result

H = ['H%d' % i for i in range(200)]
CONFIGS = [
    ('v3', {}),
    ('n_init=2', dict(n_init=2)),
    ('n_init=3', dict(n_init=3)),
    ('aim_sync', dict(aim_sync=True)),
    ('fire_best', dict(fire_best=True)),
    ('gate phit .99', dict(gate='phit', phit_min=0.99)),
    ('gate phit .999', dict(gate='phit', phit_min=0.999)),
    ('first_shot=track', dict(first_shot='track')),
    ('nis_gate 13.8', dict(nis_gate=13.8)),
    ('q_jerk 10^2', dict(q_jerk=100.0)),
    ('q_jerk 40^2', dict(q_jerk=1600.0)),
    ('singer tau2', dict(model='singer')),
    ('imm', dict(model='imm')),
    ('ambush2', dict(mode='ambush2')),
    ('ambush2+ptos', dict(mode='ambush2', shaper='ptos')),
    ('ambush2+ptos+n3', dict(mode='ambush2', shaper='ptos', n_init=3)),
    ('ptos', dict(shaper='ptos')),
    ('damped Rv5 wc15', dict(servo='damped', Rv=5.0, wc=15.0)),
    ('damped Rv10 wc10', dict(servo='damped', Rv=10.0, wc=10.0)),
    ('ambush2+fire_best', dict(mode='ambush2', fire_best=True)),
]

if __name__ == '__main__':
    which = sys.argv[1:] or [c[0] for c in CONFIGS]
    t0 = time.time()
    for label, ovr in CONFIGS:
        if label not in which:
            continue
        for setname, cases in [('std', ['A', 'B', 'C']), ('H200', H)]:
            S = summarize(run_set(cases, ovr), '%s | %s' % (label, setname))
            print(fmt(S), '  [%.0fs]' % (time.time() - t0), flush=True)
            log_result(S, extra=dict(exp='round1', ovr=str(ovr)))
