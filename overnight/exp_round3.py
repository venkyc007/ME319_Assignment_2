"""Round 3: robustness of candidates on held-out sets and off-nominal conditions."""
import sys, time
from batch import run_set, summarize, fmt, log_result

DAMP = dict(servo='damped', Rv=5.0, wc=15.0, Rv_sched=True)
CAND = {
    'v3': {},
    'A2+fb': dict(mode='ambush2', fire_best=True),
    'C1n3': dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=3, **DAMP),
}
HO = ['H%d' % i for i in range(200, 400)]
RV = ['R%d' % i for i in range(101, 201)]
TESTS = [
    ('heldout H200-399 1ms', HO, {}, {}),
    ('R101-200 1ms', RV, {}, {}),
    ('H-ho mismatch +20J+20L-20Kt', HO[:100], {}, dict(mismatch=dict(J=1.2, L=1.2, Kt=0.8))),
    ('H-ho mismatch -20J-20L+20Kt', HO[:100], {}, dict(mismatch=dict(J=0.8, L=0.8, Kt=1.2))),
    ('H-ho held valid', HO[:100], {}, dict(pulse_valid=False)),
    ('H-ho th0=20deg', HO[:100], {}, dict(th0=0.349)),
    ('H-ho Vmax 2000', HO[:100], dict(V_max=2000.0), {}),
    ('H-ho Vmax 2000 +aw', HO[:100], dict(V_max=2000.0, aw=True), {}),
    ('H-ho ctrl 5ms', HO[:100], {}, dict(ctrl_dt=0.005)),
    ('H-ho ctrl 20ms', HO[:100], {}, dict(ctrl_dt=0.02)),
    ('H-ho ctrl 50ms', HO[:100], {}, dict(ctrl_dt=0.05)),
    ('H-ho radar noise x3', HO[:100], {}, dict(sig_scale=3.0)),
    ('H-ho latency 20ms', HO[:100], {}, dict(latency=0.02)),
]

if __name__ == '__main__':
    cands = sys.argv[1:] or list(CAND)
    t0 = time.time()
    for tname, cases, povr, kw in TESTS:
        for c in cands:
            ovr = dict(CAND[c]); ovr.update(povr)
            S = summarize(run_set(cases, ovr, kw), '%s | %s' % (c, tname))
            print(fmt(S), '  [%.0fs]' % (time.time() - t0), flush=True)
            log_result(S, extra=dict(exp='round3', cand=c, test=tname))
