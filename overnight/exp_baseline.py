import sys, time
from batch import run_set, summarize, fmt, log_result

if __name__ == '__main__':
    H = ['H%d' % i for i in range(200)]
    Rc = ['R%d' % i for i in range(101, 161)]
    std = ['A', 'B', 'C']
    t0 = time.time()
    for label, cases, kw in [('v3 std 1ms', std, {}), ('v3 H200 1ms', H, {}), ('v3 R101-160 1ms', Rc, {}),
                             ('v3 H200 ctrl20ms', H, dict(ctrl_dt=0.02)), ('v3 H200 ctrl50ms', H, dict(ctrl_dt=0.05)),
                             ('v3 H200 heldvalid', H, dict(pulse_valid=False))]:
        S = summarize(run_set(cases, {}, kw), label)
        print(fmt(S), '  [%.0fs]' % (time.time() - t0), flush=True)
        log_result(S, extra=dict(exp='baseline'))
