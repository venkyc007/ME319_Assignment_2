"""Python port of guardian_v3 (turret_controller.m, turret_plant.m,
target_motion_generator.m, run_scenario.m) for fast design exploration.

Faithful to v3 by default; every new idea is behind an option in the params
dict so the baseline stays reproducible.  Not a deliverable: anything adopted
must be ported back to the .m files and re-verified in MATLAB/Octave.
"""
import math
import numpy as np
from scipy.linalg import expm

D2R = math.pi / 180


# ----------------------------------------------------------------------------
# parameters
# ----------------------------------------------------------------------------
def params(**ovr):
    P = dict(
        J_turret=1200.0, Jm=0.20, N=100.0, b=1500.0, Kt=10.0, Ke=10.0, Rm=1.0, Lm=0.10,
        theta_min=-10 * D2R, theta_max=85 * D2R,
        Ts_radar=0.05, sig_r=1.0, sig_th=0.02 * D2R, tp=0.5, R_eng=2.0, t_recharge=1.0, t_end=10.0,
        r_min=800.0, r_max=2000.0, v_max=150.0, a_max=20.0,
        q_jerk=20.0 ** 2, n_init=6,
        mode='ambush', tau_ref=0.0, fire_k=3.0, use_ff=True,
        wc=10.0, z_i=5.0, PM_target=55.0, lead_a=None, zeta_notch_den=0.7,
        newness_by_value=True, cross_fire=False, ref_wf=15.0, ref_amax=20.0, V_max=math.inf,
        # ---------------- new options (overnight exploration) -------------
        aim_hold=0.25,        # v3: re-aim when t > aim_t + aim_hold
        aim_sync=False,       # re-aim so the crossing coincides with recharge completion
        gate='ksigma',        # 'ksigma' (v3) | 'phit' (probability-of-hit)
        phit_min=0.99,
        fire_best=False,      # fire at predicted closest approach of target to beam (dt-aware)
        sweep_fire=False,     # allow firing while slewing (no extra condition) -- v3 already does
        nis_gate=None,        # chi2 threshold for innovation gating (e.g. 13.8)
        model='CA',           # 'CA' | 'singer' | 'imm'
        singer_tau=2.0,
        imm_q=(1.0, 40.0 ** 2), imm_p=0.97,
        ff_jerk=False,
        aw=False, K_aw=None,  # anti-windup (back-calculation) when V_max finite
        lat_comp=0.0,         # latency compensation (extrapolate estimate by this) [s]
        shaper='linear',      # 'linear' (v3) | 'ptos' (proximate time-optimal)
        ptos_alpha=0.8, ptos_k=15.0, ptos_kv=60.0,
        w_cross=0.0,          # PTOS: arrive with this residual relative rate [rad/s]
        first_shot=None,      # None | 'track': follow current LOS until the first pulse, then mode
        refine=True,          # ambush2: re-predict to the SAME absolute aim time each call
        grace=0.10,           # ambush2: re-aim if no pulse within grace after the aim time
        hold_after_fire=True, # ambush2: keep pointing until t+tp reaches next recharge
        servo='v3',           # 'v3' (notch-PI-lead) | 'damped' (current-feedback virtual resistance + PI-lead)
        Rv=10.0,              # virtual resistance [ohm] for servo='damped'
        notch=True,
        Rv_sched=False,       # choose Rv from the measured call period (stability table)
        disc='zoh',           # compensator discretisation: 'zoh' (v3) | 'tustin'
    )
    P.update(ovr)
    P['J_eq'] = P['J_turret'] + P['N'] ** 2 * P['Jm']
    return P


# ----------------------------------------------------------------------------
# target motion generator (exact port incl. the deterministic LCG)
# ----------------------------------------------------------------------------
def lcg(st, n, mode='u'):
    u = np.zeros(n)
    for k in range(n):
        st = (st * 1664525 + 1013904223) % 2 ** 32
        u[k] = (st + 0.5) / 2 ** 32
    if mode == 'n':
        u2 = np.zeros(n)
        for k in range(n):
            st = (st * 1664525 + 1013904223) % 2 ** 32
            u2[k] = (st + 0.5) / 2 ** 32
        u = np.sqrt(-2 * np.log(u)) * np.cos(2 * np.pi * u2)
    return u, st


def smooth_on(tg, t0, T):
    u = np.clip((tg - t0) / T, 0, 1)
    return 3 * u ** 2 - 2 * u ** 3


def integrate(tg, p0, v0, acc):
    dt = tg[1] - tg[0]
    v = np.empty((len(tg), 2)); p = np.empty((len(tg), 2))
    for j in range(2):
        v[:, j] = v0[j] + np.concatenate([[0], np.cumsum((acc[:-1, j] + acc[1:, j]) / 2 * dt)])
        p[:, j] = p0[j] + np.concatenate([[0], np.cumsum((v[:-1, j] + v[1:, j]) / 2 * dt)])
    return np.hstack([p, v, acc])


def check_env(X, tg, P, el_margin=2 * D2R):
    m = tg <= P['t_end'] + 1e-12
    r = np.hypot(X[m, 0], X[m, 1]); sp = np.hypot(X[m, 2], X[m, 3]); a = np.hypot(X[m, 4], X[m, 5])
    el = np.arctan2(X[m, 1], X[m, 0])
    return bool(np.all((r >= P['r_min']) & (r <= P['r_max'])) and np.all(sp <= P['v_max'])
                and np.all(a <= P['a_max'] + 1e-9)
                and np.all((el > P['theta_min'] + el_margin) & (el < P['theta_max'] - el_margin))
                and np.all(X[m, 1] > 0))


class Scenario:
    """Truth on a 1 ms grid + noise sequences + blackout window."""

    def __init__(self, key, P=None, X=None, bo_on=math.inf, bo_dur=0.0, noise_seed=None):
        P = P or params()
        self.Ts = P['Ts_radar']; self.sig_r = P['sig_r']; self.sig_th = P['sig_th']
        dt = 1e-3
        self.tg = np.arange(0, round((P['t_end'] + 1) / dt) + 1) * dt
        self.bo_on, self.bo_dur = bo_on, bo_dur
        self.key = key
        if X is not None:                       # externally generated truth
            self.X = X
            seed = noise_seed if noise_seed is not None else 99991
        else:
            seed = self._build(key, P)
        nK = math.ceil((P['t_end'] + 1) / self.Ts) + 2
        st2 = ((seed + 77) * 2654435761) % 2 ** 32
        self.nr, st2 = lcg(st2, nK, 'n')
        self.nth, st2 = lcg(st2, nK, 'n')

    def _build(self, key, P):
        tg = self.tg
        k0 = key[0].upper()
        if k0 in 'ABC':
            seed = ord(k0)
            p0 = [1900, 450]; v0 = [-120, 0]
            if k0 == 'A':
                acc = np.zeros((len(tg), 2))
            else:
                w = smooth_on(tg, 1.0, 0.5)
                acc = np.column_stack([12 * w * np.sin(1.2 * (tg - 1)), 12 * w * np.sin(1.6 * (tg - 1))])
            if k0 == 'C':
                self.bo_on, self.bo_dur = 1.0, 0.5
            X = integrate(tg, p0, v0, acc)
        elif k0 == 'R':
            seed = 1000 + int(key[1:])
            st = (seed * 2654435761) % 2 ** 32
            ok = False; tries = 0
            while not ok:
                tries += 1
                u1, st = lcg(st, 2)
                r0 = 1400 + 550 * u1[0]; el0 = (3 + 50 * u1[1]) * D2R
                p0 = r0 * np.array([math.cos(el0), math.sin(el0)])
                u1, st = lcg(st, 1); spd = 60 + 85 * u1[0]
                u1, st = lcg(st, 1); hdg = math.pi + (-0.6 + 1.2 * u1[0])
                v0 = spd * np.array([math.cos(hdg), math.sin(hdg)])
                u1, st = lcg(st, 1); t_on = 0.5 + 2.5 * u1[0]
                u1, st = lcg(st, 1); w = smooth_on(tg, t_on, 0.3 + 0.5 * u1[0])
                tf_, st = lcg(st, 4); tp_, st = lcg(st, 4); ta_, st = lcg(st, 4)
                # MATLAB reshape(.,2,2) is column-major
                f = 0.4 + 2.0 * tf_.reshape(2, 2, order='F'); ph = 2 * np.pi * tp_.reshape(2, 2, order='F')
                amp = ta_.reshape(2, 2, order='F')
                ax = amp[0, 0] * np.sin(f[0, 0] * tg + ph[0, 0]) + amp[0, 1] * np.sin(f[0, 1] * tg + ph[0, 1])
                ay = amp[1, 0] * np.sin(f[1, 0] * tg + ph[1, 0]) + amp[1, 1] * np.sin(f[1, 1] * tg + ph[1, 1])
                acc = np.column_stack([ax, ay])
                u1, st = lcg(st, 1)
                acc = acc / np.max(np.hypot(acc[:, 0], acc[:, 1])) * (8 + 11.5 * u1[0])
                acc = acc * w[:, None]
                X = integrate(tg, p0, v0, acc)
                ok = check_env(X, tg, P)
                if tries > 200:
                    raise RuntimeError('no admissible case')
            u1, st = lcg(st, 1); self.bo_on = 1.0 + 6 * u1[0]
            u1, st = lcg(st, 1); self.bo_dur = 0.5 * u1[0]
        else:
            raise ValueError(key)
        self.X = X
        return seed

    def truth(self, t):
        dt = 1e-3
        u = min(max(t, 0), self.tg[-1]) / dt
        j = min(int(math.floor(u + 1e-9)), len(self.tg) - 2)
        w = u - j
        return (1 - w) * self.X[j] + w * self.X[j + 1]

    def in_bo(self, tt):
        return (tt >= self.bo_on - 1e-9) and (tt < self.bo_on + self.bo_dur - 1e-9)

    def radar(self, t, latency=0.0, sig_scale=1.0):
        """Same convention as the .m generator: meas_valid only on the 50 ms grid,
        value held otherwise.  latency: measurement describes truth at t-latency."""
        Ts = self.Ts
        k = int(math.floor(t / Ts + 1e-9))
        on_grid = abs(t - k * Ts) < 1e-6
        valid = on_grid and not self.in_bo(k * Ts)
        kk = k
        while kk >= 0 and self.in_bo(kk * Ts):
            kk -= 1
        kk = max(kk, 0)
        Xk = self.truth(max(kk * Ts - latency, 0.0))
        r = math.hypot(Xk[0], Xk[1]) + self.sig_r * sig_scale * self.nr[kk]
        th = math.atan2(Xk[1], Xk[0]) + self.sig_th * sig_scale * self.nth[kk]
        return r, th, valid


# ----------------------------------------------------------------------------
# compensator design (port of design_controller.m)
# ----------------------------------------------------------------------------
def design(P):
    numP = np.array([P['N'] * P['Kt']])
    Reff = P['Rm'] + (P['Rv'] if P.get('servo') == 'damped' else 0.0)
    q = np.polyadd(np.polymul([P['J_eq'], P['b']], [P['Lm'], Reff]), [P['N'] ** 2 * P['Kt'] * P['Ke']])
    denP = np.polymul([1, 0], q)
    qn = q / q[0]
    wn = math.sqrt(qn[2]); zp = qn[1] / (2 * wn)
    notch_num = np.array([1, 2 * zp * wn, wn ** 2]); notch_den = np.array([1, 2 * P['zeta_notch_den'] * wn, wn ** 2])
    if not P.get('notch', True) or P.get('servo') == 'damped':
        notch_num = np.array([1.0]); notch_den = np.array([1.0])
    pi_num = np.array([1, P['z_i']]); pi_den = np.array([1, 0])
    wm = P['wc']; jw = 1j * wm
    ph = lambda c: np.angle(np.polyval(c, jw))
    phi_rest = (ph(notch_num) - ph(notch_den) + ph(pi_num) - math.pi / 2 + ph(numP) - ph([1, 0]) - ph(qn)) * 180 / math.pi
    phi_m = P['PM_target'] - (180 + phi_rest)
    a = (1 + math.sin(phi_m * D2R)) / (1 - math.sin(phi_m * D2R)) if phi_m > 0 else 1.0
    if P.get('lead_a'):
        a = P['lead_a']
    lead_num = np.array([1 / (wm / math.sqrt(a)), 1]); lead_den = np.array([1 / (wm * math.sqrt(a)), 1])
    numC0 = np.polymul(np.polymul(notch_num, pi_num), lead_num)
    denC0 = np.polymul(np.polymul(notch_den, pi_den), lead_den)
    Lmag = abs(np.polyval(numC0, jw) / np.polyval(denC0, jw) * np.polyval(numP, jw) / np.polyval(denP, jw))
    K = 1 / Lmag
    numC = K * numC0; denC = denC0
    A, B, Cc, Dd = tf2ss(numC, denC)
    C = dict(numP=numP, denP=denP, wn=wn, zeta_p=zp, lead_a=a, K=K, numC=numC, denC=denC,
             A=A, B=B, Cc=Cc, D=Dd,
             ff_v=(P['b'] * Reff + P['N'] ** 2 * P['Kt'] * P['Ke']) / (P['N'] * P['Kt']),
             ff_a=(P['J_eq'] * Reff + P['b'] * P['Lm']) / (P['N'] * P['Kt']), phi_lead=phi_m,
             ff_j=(P['J_eq'] * P['Lm']) / (P['N'] * P['Kt']))
    return C


def tf2ss(num, den):
    num = np.asarray(num, float) / den[0]; den = np.asarray(den, float) / den[0]
    n = len(den) - 1
    num = np.concatenate([np.zeros(n + 1 - len(num)), num])
    D = num[0]
    r = num[1:] - D * den[1:]
    A = np.zeros((n, n)); A[:-1, 1:] = np.eye(n - 1); A[-1, :] = -den[1:][::-1]
    B = np.zeros(n); B[-1] = 1
    Cc = r[::-1].copy()
    return A, B, Cc, D


def zoh(A, B, dt):
    n = A.shape[0]
    M = np.zeros((n + 1, n + 1)); M[:n, :n] = A; M[:n, n] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n]


# ----------------------------------------------------------------------------
# estimator pieces
# ----------------------------------------------------------------------------
def ca_F(dt):
    F = np.eye(6)
    F[0, 2] = F[1, 3] = F[2, 4] = F[3, 5] = dt
    F[0, 4] = F[1, 5] = dt * dt / 2
    return F


def ca_Q(dt, q):
    Q1 = q * np.array([[dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6], [dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2],
                       [dt ** 3 / 6, dt ** 2 / 2, dt]])
    return np.kron(Q1, np.eye(2))


def singer_FQ(dt, tau, sig_a):
    """Singer (first-order Gauss-Markov acceleration) model, exact discretisation."""
    a = 1 / tau
    Ac = np.zeros((6, 6)); Ac[0, 2] = Ac[1, 3] = Ac[2, 4] = Ac[3, 5] = 1; Ac[4, 4] = Ac[5, 5] = -a
    Gq = np.zeros((6, 6)); Gq[4, 4] = Gq[5, 5] = 2 * a * sig_a ** 2
    n = 6
    M = np.zeros((2 * n, 2 * n)); M[:n, :n] = -Ac; M[:n, n:] = Gq; M[n:, n:] = Ac.T
    E = expm(M * dt)
    F = E[n:, n:].T
    Q = F @ E[:n, n:]
    return F, 0.5 * (Q + Q.T)


def h_meas(x):
    px, py = x[0], x[1]; r = math.hypot(px, py)
    H = np.zeros((2, 6))
    H[0, 0] = px / r; H[0, 1] = py / r; H[1, 0] = -py / r ** 2; H[1, 1] = px / r ** 2
    return np.array([r, math.atan2(py, px)]), H


def los_angle(x):
    px, py, vx, vy, ax, ay = x
    r2 = px * px + py * py
    th = math.atan2(py, px)
    c = px * vy - py * vx
    thd = c / r2
    thdd = (px * ay - py * ax) / r2 - 2 * (px * vx + py * vy) * c / r2 ** 2
    return th, thd, thdd


# ----------------------------------------------------------------------------
# controller (port of turret_controller.m with options)
# ----------------------------------------------------------------------------
class Controller:
    def __init__(self, P):
        self.P = P
        self.C = design(P)
        self.last_t = 0.0
        self.init = False
        self.n_meas = 0
        self.x = np.zeros(6); self.Pk = np.eye(6)
        nC = self.C['A'].shape[0]
        self.xc = np.zeros(nC); self.e_prev = 0.0; self.dt_c = -1
        self.last_fire_t = -math.inf; self.fire_prev = 0
        self.aim_t = math.nan; self.aim_th = math.nan; self.fired_on_aim = False
        self.z_last = (math.nan, math.nan); self.n_fired = 0
        self.ref_init = False; self.th_r = 0.0; self.w_r = 0.0
        self.m_prev = math.nan; self.V_last = 0.0
        self._fq_cache = {}
        self.u_sat_prev = 0.0; self.u_prev = 0.0
        self.i_hat = 0.0; self.thd_prev = 0.0
        # IMM
        if P['model'] == 'imm':
            self.mu = np.array([0.5, 0.5])
            self.xs = [np.zeros(6), np.zeros(6)]; self.Ps = [np.eye(6), np.eye(6)]
        self.dbg = {}

    # ---- process model
    def FQ(self, dt, q=None):
        P = self.P
        key = (round(dt, 9), q)
        if key in self._fq_cache:
            return self._fq_cache[key]
        if P['model'] == 'singer':
            F, Q = singer_FQ(dt, P['singer_tau'], math.sqrt(P['q_jerk']) * P['singer_tau'] / 2)
        else:
            F, Q = ca_F(dt), ca_Q(dt, P['q_jerk'] if q is None else q)
        if len(self._fq_cache) < 200:
            self._fq_cache[key] = (F, Q)
        return F, Q

    def predict(self, dt):
        P = self.P
        if P['model'] == 'imm':
            # interaction (mixing) only at measurement times; here pure propagation
            for j in range(2):
                F, Q = self.FQ(dt, P['imm_q'][j])
                self.xs[j] = F @ self.xs[j]; self.Ps[j] = F @ self.Ps[j] @ F.T + Q
            self._imm_combine()
        else:
            F, Q = self.FQ(dt)
            self.x = F @ self.x; self.Pk = F @ self.Pk @ F.T + Q

    def _imm_combine(self):
        x = self.mu[0] * self.xs[0] + self.mu[1] * self.xs[1]
        Pk = np.zeros((6, 6))
        for j in range(2):
            d = self.xs[j] - x
            Pk += self.mu[j] * (self.Ps[j] + np.outer(d, d))
        self.x, self.Pk = x, Pk

    def update(self, z, Rz):
        P = self.P
        if P['model'] == 'imm':
            p = P['imm_p']; Pi = np.array([[p, 1 - p], [1 - p, p]])
            cbar = Pi.T @ self.mu
            mix = (Pi * self.mu[:, None]) / cbar[None, :]   # mix[i,j] = P(i | j)
            xs0, Ps0 = [], []
            for j in range(2):
                x0 = mix[0, j] * self.xs[0] + mix[1, j] * self.xs[1]
                P0 = np.zeros((6, 6))
                for i in range(2):
                    d = self.xs[i] - x0
                    P0 += mix[i, j] * (self.Ps[i] + np.outer(d, d))
                xs0.append(x0); Ps0.append(P0)
            lik = np.zeros(2)
            for j in range(2):
                xj, Pj, l, _ = ekf_update(xs0[j], Ps0[j], z, Rz, P['nis_gate'])
                self.xs[j], self.Ps[j] = xj, Pj; lik[j] = l
            mu = lik * cbar
            self.mu = mu / mu.sum() if mu.sum() > 0 else cbar
            self._imm_combine()
            return True
        x, Pk, _, acc = ekf_update(self.x, self.Pk, z, Rz, P['nis_gate'])
        self.x, self.Pk = x, Pk
        return acc

    def __call__(self, t, r_meas, th_meas, meas_valid, th_t, thd_t):
        P, C = self.P, self.C
        if t < self.last_t - 1e-9:
            return self.V_last, 0
        dt = max(t - self.last_t, 0.0)
        self.last_t = t
        if dt > 0:
            if P.get('observer', 'deadbeat') == 'deadbeat':
                # exact one-step reconstruction from the ZOH plant model (V held over dt)
                if abs(dt - getattr(self, 'dt_obs', -1)) > 1e-12:
                    self.Phi, self.Gam = plant_zoh(P, dt); self.dt_obs = dt
                Phi, Gam = self.Phi, self.Gam
                i_prev = (thd_t - Phi[1, 1] * self.thd_prev - Gam[1] * self.V_last) / Phi[1, 2]
                self.i_hat = Phi[2, 1] * self.thd_prev + Phi[2, 2] * i_prev + Gam[2] * self.V_last
            else:
                a = math.exp(-P['Rm'] * dt / P['Lm'])
                wavg = 0.5 * (self.thd_prev + thd_t)
                self.i_hat = a * self.i_hat + (1 - a) * (self.V_last - P['Ke'] * P['N'] * wavg) / P['Rm']
        self.thd_prev = thd_t
        if P['Rv_sched'] and dt > 0 and abs(dt - getattr(self, 'dt_design', -1)) > 0.2 * dt:
            self.dt_design = dt
            if dt > P.get('coarse_dt', 0.03):
                # slow evaluator: fall back to the v3 servo (notch-PI-lead, wc 10) + linear shaper
                self.P = P = dict(P, servo='v3', Rv=0.0, wc=10.0, shaper='linear')
            else:
                Rv = 5.0 if dt <= 0.021 else 3.0
                self.P = P = dict(P, Rv=Rv)
            self.C = C = design(P); self.dt_c = -1
            if self.xc.shape[0] != C['A'].shape[0]:
                self.xc = np.zeros(C['A'].shape[0])
        if self.init and dt > 0:
            self.predict(dt)
        ok = meas_valid and math.isfinite(r_meas) and math.isfinite(th_meas)
        if P['newness_by_value']:
            is_new = ok and not (r_meas == self.z_last[0] and th_meas == self.z_last[1])
        else:
            is_new = ok
        if is_new:
            self.z_last = (r_meas, th_meas)
            z = np.array([r_meas, th_meas])
            Rz = np.diag([P['sig_r'] ** 2, P['sig_th'] ** 2])
            if not self.init:
                c, s = math.cos(th_meas), math.sin(th_meas)
                self.x = np.array([r_meas * c, r_meas * s, 0, 0, 0, 0.0])
                Jp = np.array([[c, -r_meas * s], [s, r_meas * c]])
                Pk = np.zeros((6, 6)); Pk[:2, :2] = Jp @ Rz @ Jp.T
                Pk[2:4, 2:4] = P['v_max'] ** 2 * np.eye(2); Pk[4:, 4:] = P['a_max'] ** 2 * np.eye(2)
                self.Pk = Pk
                if P['model'] == 'imm':
                    self.xs = [self.x.copy(), self.x.copy()]; self.Ps = [Pk.copy(), Pk.copy()]
                self.init = True
            else:
                self.update(z, Rz)
            self.n_meas += 1

        V = 0.0; fire = 0
        if not self.init:
            self.fire_prev = 0
            return V, fire
        conv = self.n_meas >= P['n_init']
        xe = self.x
        if P['lat_comp'] > 0:
            xe = ca_F(P['lat_comp']) @ self.x
        xp = ca_F(P['tp']) @ xe
        mode = P['mode']
        if mode == 'hybrid':
            mode = 'ambush' if self.n_fired == 0 else 'track'
        if P['first_shot'] == 'track' and self.n_fired == 0:
            mode = 'track_now'
        if mode == 'ambush':
            if not conv:
                th_ref, thd_ref, thdd_ref = los_angle(xe)
            else:
                if P['aim_sync']:
                    # next shot possible at tr = last_fire + recharge; aim at the
                    # state predicted tp ahead, but never before t + tp - (latest re-aim)
                    need = (math.isnan(self.aim_t) or t > self.aim_t + P['aim_hold'] or self.fired_on_aim)
                    if self.fired_on_aim and (t + P['tp'] < self.last_fire_t + P['t_recharge']):
                        need = False   # wait: an aim point earlier than recharge is useless
                    if need:
                        self.aim_t = t + P['tp']; self.aim_th = los_angle(xp)[0]; self.fired_on_aim = False
                else:
                    if math.isnan(self.aim_t) or t > self.aim_t + P['aim_hold'] or self.fired_on_aim:
                        self.aim_t = t + P['tp']; self.aim_th = los_angle(xp)[0]; self.fired_on_aim = False
                th_ref, thd_ref, thdd_ref = self.aim_th, 0.0, 0.0
        elif mode == 'ambush2':
            if not conv:
                th_ref, thd_ref, thdd_ref = los_angle(xe)
            else:
                t_ready = self.last_fire_t + P['t_recharge']
                if self.fired_on_aim:
                    # place the next aim point only when its horizon tp reaches recharge completion
                    if (not P['hold_after_fire']) or t + P['tp'] >= t_ready - 1e-9:
                        self.aim_t = t + P['tp']; self.fired_on_aim = False
                elif math.isnan(self.aim_t) or t > self.aim_t + P['grace']:
                    self.aim_t = t + P['tp']
                if not self.fired_on_aim:
                    tau = max(self.aim_t - t, 0.0) if P['refine'] else None
                    if P['refine']:
                        self.aim_th = los_angle(ca_F(tau) @ xe)[0]
                    elif abs(self.aim_t - t - P['tp']) < 1e-9:
                        self.aim_th = los_angle(xp)[0]
                th_ref, thd_ref, thdd_ref = self.aim_th, 0.0, 0.0
        elif mode == 'track_now':
            th_ref, thd_ref, thdd_ref = los_angle(xe)
        else:
            th_ref, thd_ref, thdd_ref = los_angle(ca_F(P['tau_ref']) @ xe)
        th_ref = min(max(th_ref, P['theta_min'] + D2R), P['theta_max'] - D2R)
        a_r = 0.0
        if P['ref_wf'] > 0:
            if not self.ref_init:
                self.th_r, self.w_r, self.ref_init = th_t, thd_t, True
            if dt > 0:
                h = min(dt, 0.002 if P["shaper"] == "ptos" else 0.01); nsub = max(1, round(dt / h)); h = dt / nsub
                wf = P['ref_wf']
                am = P['ref_amax']
                for _ in range(nsub):
                    if P['shaper'] == 'ptos':
                        ee = th_ref - self.th_r
                        ae = abs(ee); aa = P['ptos_alpha'] * am; kl = P['ptos_k']
                        el = aa / kl ** 2          # C1 junction of linear and sqrt zones
                        if ae > el:
                            fv = math.sqrt(2 * aa * (ae - el / 2))
                        else:
                            fv = kl * ae
                        v_des = thd_ref + math.copysign(fv, ee)
                        a_r = thdd_ref + P['ptos_kv'] * (v_des - self.w_r)
                    else:
                        a_r = thdd_ref + 2 * wf * (thd_ref - self.w_r) + wf ** 2 * (th_ref - self.th_r)
                    a_r = min(max(a_r, -am), am)
                    vm = P.get('ref_vmax', math.inf)   # drive-aware speed cap (realism study)
                    if math.isfinite(vm):
                        a_r = min(max(a_r, (-vm - self.w_r) / h), (vm - self.w_r) / h)
                    self.th_r += h * self.w_r + 0.5 * h * h * a_r; self.w_r += h * a_r
            th_ref, thd_ref, thdd_ref = self.th_r, self.w_r, a_r
        e = th_ref - th_t
        if dt > 0:
            if abs(dt - self.dt_c) > 1e-12:
                if P['disc'] == 'tustin':
                    n_ = C['A'].shape[0]; I_ = np.eye(n_)
                    M_ = np.linalg.inv(I_ - C['A'] * dt / 2)
                    self.Ad = M_ @ (I_ + C['A'] * dt / 2); self.Bd = M_ @ C['B'] * dt
                    self.Ct = C['Cc'] @ M_; self.Dt = C['D'] + float(C['Cc'] @ M_ @ C['B']) * dt / 2
                else:
                    self.Ad, self.Bd = zoh(C['A'], C['B'], dt)
                self.dt_c = dt
            ein = self.e_prev
            if P['aw'] and math.isfinite(P['V_max']):
                # back-calculation: correct the error fed to Gc by the saturation excess
                kaw = P['K_aw'] if P['K_aw'] is not None else 1.0 / max(abs(C['D']), 1e-9)
                ein = ein + kaw * (self.u_sat_prev - self.u_prev)
            self.xc = self.Ad @ self.xc + self.Bd * ein
        if P['disc'] == 'tustin' and dt > 0:
            # bilinear: x(k+1) = Ad x(k) + Bd e(k), u = Ct x(k) + Dt e(k)  (state already advanced with e_prev)
            u_fb = float(C['Cc'] @ self.xc + C['D'] * e)
        else:
            u_fb = float(C['Cc'] @ self.xc + C['D'] * e)
        self.e_prev = e
        u_ff = 0.0
        if P['use_ff']:
            u_ff = C['ff_v'] * thd_ref + C['ff_a'] * thdd_ref
            if P['ff_jerk'] and dt > 0:
                jerk = (thdd_ref - getattr(self, 'a_ref_prev', 0.0)) / dt
                u_ff += P.get('ff_jerk_gain', 1.0) * C['ff_j'] * jerk
            self.a_ref_prev = thdd_ref
        u = u_fb + u_ff
        if P['servo'] == 'damped':
            u -= P['Rv'] * self.i_hat
        V = min(max(u, -P['V_max']), P['V_max'])
        self.u_prev, self.u_sat_prev = u, V

        # ---- fire decision
        g = np.array([-math.sin(th_t), math.cos(th_t)])
        bdir = np.array([math.cos(th_t), math.sin(th_t)])
        m_s = float(g @ xe[:2]); miss = abs(m_s)
        sig = math.sqrt(max(float(g @ self.Pk[:2, :2] @ g), 0.0))
        in_front = float(bdir @ xe[:2]) > 0
        ready = (t - self.last_fire_t) >= P['t_recharge'] + 1e-6
        if P['gate'] == 'phit':
            ok_gate = phit(miss, sig, P['R_eng']) >= P['phit_min']
        else:
            ok_gate = miss + P['fire_k'] * sig <= P['R_eng']
        want = conv and in_front and ready and ok_gate
        if P['fire_best'] and want and dt > 0:
            # beam-normal rate of target relative to the rotating beam; if the
            # offset will be smaller at the next call (assumed dt later), wait.
            md = float(g @ xe[2:4]) - thd_t * float(bdir @ xe[:2])
            m_next = m_s + md * dt
            if abs(m_next) < abs(m_s) - 0.05:
                # will it still pass the gate next call? only wait if yes
                if P['gate'] == 'phit':
                    ok_next = phit(abs(m_next), sig, P['R_eng']) >= P['phit_min']
                else:
                    ok_next = abs(m_next) + P['fire_k'] * sig <= P['R_eng']
                if ok_next:
                    want = False
        if P['cross_fire'] and not want and conv and in_front and ready and dt > 0:
            md = float(g @ xe[2:4]) - thd_t * float(bdir @ xe[:2])
            m_next = m_s + md * dt
            best_now = abs(m_s) <= abs(m_next) and np.sign(m_next) != np.sign(m_s)
            best_now = best_now or (abs(m_s) <= abs(m_next) and abs(self.m_prev) > abs(m_s)
                                    and np.sign(self.m_prev) == np.sign(m_s))
            want = best_now and (abs(m_s) + P['fire_k'] * sig <= P['R_eng'])
        self.m_prev = m_s
        if want and self.fire_prev == 0:
            fire = 1; self.last_fire_t = t; self.fired_on_aim = True; self.n_fired += 1
        self.fire_prev = fire
        self.V_last = V
        self.dbg = dict(th_ref=th_ref, miss_est=miss, sig=sig, conv=conv, xp=xp)
        return V, fire


def phit(m, s, R):
    if s <= 1e-12:
        return 1.0 if m <= R else 0.0
    from math import erf, sqrt
    Phi = lambda z: 0.5 * (1 + erf(z / sqrt(2)))
    return Phi((R - m) / s) - Phi((-R - m) / s)


def ekf_update(x, Pk, z, Rz, nis_gate=None):
    h, H = h_meas(x)
    y = z - h; y[1] = math.atan2(math.sin(y[1]), math.cos(y[1]))
    S = H @ Pk @ H.T + Rz
    Si = np.linalg.inv(S)
    nis = float(y @ Si @ y)
    lik = math.exp(-0.5 * nis) / (2 * math.pi * math.sqrt(np.linalg.det(S)))
    if nis_gate is not None and nis > nis_gate:
        return x, Pk, lik, False
    K = Pk @ H.T @ Si
    x = x + K @ y
    IKH = np.eye(6) - K @ H
    Pk = IKH @ Pk @ IKH.T + K @ Rz @ K.T
    return x, Pk, lik, True


# ----------------------------------------------------------------------------
# plant + harness
# ----------------------------------------------------------------------------
def plant(V, x, P, ext=None):
    th, thd, i = x
    tau = 0.0
    if th > P['theta_max']:
        tau = 1e7 * (th - P['theta_max']) + 1e5 * max(thd, 0)
    elif th < P['theta_min']:
        tau = 1e7 * (th - P['theta_min']) + 1e5 * min(thd, 0)
    J = P['J_eq']; NKt = P['N'] * P['Kt']; Lm = P['Lm']
    if ext:   # plant mismatch for robustness tests
        J *= ext.get('J', 1); NKt *= ext.get('Kt', 1); Lm *= ext.get('L', 1)
    return np.array([thd, (NKt * i - P['b'] * thd - tau) / J,
                     (V - P['Rm'] * i - P['Ke'] * P['N'] * thd) / Lm])


def plant_zoh(P, dt):
    J = P['J_eq']
    A = np.array([[0, 1, 0], [0, -P['b'] / J, P['N'] * P['Kt'] / J],
                  [0, -P['Ke'] * P['N'] / P['Lm'], -P['Rm'] / P['Lm']]])
    B = np.array([0, 0, 1 / P['Lm']])
    return zoh(A, B, dt)


def ray_miss(th, p):
    s = math.cos(th) * p[0] + math.sin(th) * p[1]
    if s < 0:
        return math.hypot(p[0], p[1])
    return abs(-math.sin(th) * p[0] + math.cos(th) * p[1])


def simulate(sc, P=None, dt=1e-3, ctrl_dt=None, latency=0.0, sig_scale=1.0, mismatch=None,
             pulse_valid=True, log=False, th0=0.0):
    """Run one engagement.  ctrl_dt: controller called only every ctrl_dt (ZOH on V),
    plant still integrated with RK4 at dt.  pulse_valid=False: evaluator holds
    meas_valid high between radar samples."""
    P = P or params()
    ctl = Controller(P)
    n = int(round(P['t_end'] / dt)) + 1
    x = np.array([th0, 0.0, 0.0])
    shots = []
    fire_prev = 0; last_pulse = -math.inf
    V = 0.0; fc = 0
    every = 1 if ctrl_dt is None else max(1, int(round(ctrl_dt / dt)))
    L = {k: np.zeros(n) for k in ('t', 'th', 'thd', 'i', 'V', 'th_ref', 'th_true', 'r_true', 'miss', 'sig', 'fire')} if log else None
    maxV = 0.0; maxI = 0.0
    for k in range(n):
        t = k * dt
        X = sc.truth(t)
        if k % every == 0:
            rm, thm, mv = sc.radar(t, latency, sig_scale)
            if not pulse_valid:
                mv = not sc.in_bo(math.floor(t / sc.Ts + 1e-9) * sc.Ts)
            V, fc = ctl(t, rm, thm, mv, x[0], x[1])
        else:
            fc = 0 if fc == 1 and False else fc
        if fc == 1 and fire_prev == 0 and t - last_pulse >= P['t_recharge'] - 1e-9:
            last_pulse = t
            d = ray_miss(x[0], X[:2])
            shots.append((t, d, d <= P['R_eng']))
        fire_prev = fc
        maxV = max(maxV, abs(V)); maxI = max(maxI, abs(x[2]))
        if log:
            L['t'][k] = t; L['th'][k] = x[0]; L['thd'][k] = x[1]; L['i'][k] = x[2]; L['V'][k] = V
            L['th_ref'][k] = ctl.dbg.get('th_ref', math.nan); L['th_true'][k] = math.atan2(X[1], X[0])
            L['r_true'][k] = math.hypot(X[0], X[1]); L['miss'][k] = ray_miss(x[0], X[:2])
            L['sig'][k] = ctl.dbg.get('sig', math.nan); L['fire'][k] = fc
        if k < n - 1:
            f = lambda xx: plant(V, xx, P, mismatch)
            k1 = f(x); k2 = f(x + dt / 2 * k1); k3 = f(x + dt / 2 * k2); k4 = f(x + dt * k3)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    shots = np.array(shots, dtype=float).reshape(-1, 3)
    hits = shots[:, 2] > 0 if len(shots) else np.zeros(0, bool)
    R = dict(key=sc.key, n_shots=len(shots), n_hits=int(hits.sum()),
             t1=float(shots[hits][0, 0]) if hits.any() else math.nan,
             miss_all=float(shots[:, 1].mean()) if len(shots) else math.nan,
             miss_hits=float(shots[hits, 1].mean()) if hits.any() else math.nan,
             miss_first=float(shots[hits][0, 1]) if hits.any() else math.nan,
             maxV=maxV, maxI=maxI, shots=shots)
    if log:
        R['log'] = L
    return R
