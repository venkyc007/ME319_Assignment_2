"""Discrete-time closed-loop analysis of the v3 loop vs controller call period.
Plant: ZOH-equivalent of Gp(s) (V held between calls).  Controller: v3 implementation
(ZOH of the error into the compensator state + direct feedthrough), or alternatives."""
import numpy as np, math
from scipy.signal import cont2discrete, ss2tf, tf2ss as sp_tf2ss
import gsim


def plant_ss(P):
    # x = [th, thd, i]
    J = P['J_eq']; A = np.array([[0, 1, 0], [0, -P['b'] / J, P['N'] * P['Kt'] / J],
                                 [0, -P['Ke'] * P['N'] / P['Lm'], -P['Rm'] / P['Lm']]])
    B = np.array([[0], [0], [1 / P['Lm']]]); C = np.array([[1, 0, 0]]); D = np.array([[0.0]])
    return A, B, C, D


def ctrl_d(P, dt, method='v3'):
    C = gsim.design(P)
    if method == 'v3':      # xc+ = Ad xc + Bd e_prev  (one-step delay on e into the state), u = Cc xc + D e
        Ad, Bd = gsim.zoh(C['A'], C['B'], dt)
        # write as standard ss with input e(k): augment state with e_prev
        n = Ad.shape[0]
        A = np.zeros((n + 1, n + 1)); A[:n, :n] = Ad; A[:n, n] = Bd
        B = np.zeros((n + 1, 1)); B[n, 0] = 1
        Cm = np.zeros((1, n + 1)); Cm[0, :n] = C['Cc']
        return A, B, Cm, np.array([[C['D']]])
    else:
        Ac, Bc, Cc, Dc = C['A'], C['B'][:, None], C['Cc'][None, :], np.array([[C['D']]])
        kw = {}
        if method == 'tustin_pw':
            kw = dict(alpha=None)
            Ad, Bd, Cd, Dd, _ = cont2discrete((Ac, Bc, Cc, Dc), dt, method='bilinear')
        else:
            Ad, Bd, Cd, Dd, _ = cont2discrete((Ac, Bc, Cc, Dc), dt, method=method)
        return Ad, Bd, Cd, Dd


def closed_loop_poles(P, dt, method='v3', Pplant=None):
    Ap, Bp, Cp, Dp = plant_ss(Pplant or P)
    Apd, Bpd, Cpd, Dpd, _ = cont2discrete((Ap, Bp, Cp, Dp), dt, method='zoh')
    Ac, Bc, Cc, Dc = ctrl_d(P, dt, method)
    # u = Cc xc + Dc (r - y); y = Cpd xp ; xp+ = Apd xp + Bpd u ; xc+ = Ac xc + Bc (r - y)
    n1, n2 = Apd.shape[0], Ac.shape[0]
    A = np.zeros((n1 + n2, n1 + n2))
    A[:n1, :n1] = Apd - Bpd @ Dc @ Cpd; A[:n1, n1:] = Bpd @ Cc
    A[n1:, :n1] = -Bc @ Cpd; A[n1:, n1:] = Ac
    return np.linalg.eigvals(A)


def describe(z, dt):
    z = z[np.abs(z) > 1e-9]
    s = np.log(z.astype(complex)) / dt
    wn = np.abs(s); zeta = -s.real / np.maximum(wn, 1e-12)
    rho = np.max(np.abs(z))
    i = np.argmin(np.where(wn > 1, zeta, 9))
    return rho, zeta[i], wn[i]


if __name__ == '__main__':
    P = gsim.params()
    for method in ['v3', 'bilinear', 'zoh']:
        print('--- controller discretisation:', method)
        for dt in [0.001, 0.005, 0.01, 0.02, 0.03, 0.04, 0.05]:
            z = closed_loop_poles(P, dt, method)
            rho, zmin, wn = describe(z, dt)
            print('dt=%5.3f  max|z|=%.4f  min damping=%.3f at %.1f rad/s' % (dt, rho, zmin, wn))


def cl_poles_damped(P, dt, Pplant=None):
    """servo='damped': u = Cc xc + D e - Rv*i (i assumed known at samples), ZOH plant."""
    Ap, Bp, Cp, Dp = plant_ss(Pplant or P)
    Apd, Bpd, _, _, _ = cont2discrete((Ap, Bp, Cp, Dp), dt, method='zoh')
    C = gsim.design(P)
    Ac, Bc = gsim.zoh(C['A'], C['B'], dt); Bc = Bc[:, None]
    Cc = C['Cc'][None, :]; Dc = C['D']
    K = np.array([[Dc, 0, P['Rv'] if P['servo'] == 'damped' else 0.0]])   # u = Cc xc - K xp  (r = 0)
    n1, n2 = 3, Ac.shape[0]
    A = np.zeros((n1 + n2, n1 + n2))
    A[:n1, :n1] = Apd - Bpd @ K; A[:n1, n1:] = Bpd @ Cc
    A[n1:, :n1] = -Bc @ np.array([[1, 0, 0]]); A[n1:, n1:] = Ac
    return np.linalg.eigvals(A)


def scan_damped():
    for Rv in [0, 3, 5, 10, 20]:
        for wc in [10, 15, 20]:
            P = gsim.params(servo='damped' if Rv else 'v3', Rv=Rv, wc=wc)
            row = []
            for dt in [0.001, 0.005, 0.01, 0.02, 0.05]:
                rho, zmin, wn = describe(cl_poles_damped(P, dt), dt)
                row.append('%.3f/%.2f' % (rho, zmin))
            C = gsim.design(P)
            print('Rv=%2d wc=%2d lead_phi=%5.1f  ' % (Rv, wc, C['phi_lead']) + '  '.join(row))
