"""Figures for REPORT.md (static PNGs)."""
import os, json, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import gsim, batch, discrete_analysis as da

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figs')
os.makedirs(OUT, exist_ok=True)
C_V3, C_V4, INK, MUTED, GRID = '#eb6834', '#2a78d6', '#1f1f1d', '#6b6a64', '#e1e0d9'
plt.rcParams.update({'font.size': 10, 'axes.edgecolor': '#c3c2b7', 'axes.labelcolor': INK,
                     'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.grid': True, 'grid.color': GRID,
                     'grid.linewidth': 0.6, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb', 'lines.linewidth': 2})
DAMP = dict(servo='damped', Rv=5.0, wc=15.0, Rv_sched=True)
V4 = dict(mode='ambush2', fire_best=True, shaper='ptos', n_init=3, **DAMP)


def fig_damping():
    dts = np.array([0.001, 0.002, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05])
    z3, z4 = [], []
    for dt in dts:
        z3.append(da.describe(da.cl_poles_damped(gsim.params(), dt), dt)[1])
        if dt > 0.03:      # v4 falls back to the v3 servo for slow evaluators
            z4.append(z3[-1]); continue
        Rv = 5.0 if dt <= 0.021 else 3.0
        z4.append(da.describe(da.cl_poles_damped(gsim.params(servo='damped', Rv=Rv, wc=15.0), dt), dt)[1])
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.plot(dts * 1000, z3, color=C_V3, marker='o', ms=5, label='v3: notch + PI + lead')
    ax.plot(dts * 1000, z4, color=C_V4, marker='o', ms=5, label='v4: current feedback (Rv) + PI-lead')
    ax.axhline(0, color='#c3c2b7', lw=1)
    ax.axvspan(30, 52, color='#e1e0d9', alpha=0.5, lw=0); ax.text(41, 0.5, 'v4 falls back\nto v3 servo', ha='center', color=MUTED, fontsize=8)
    ax.set_xlabel('controller call period [ms]'); ax.set_ylabel('least-damped closed-loop pole, ζ')
    ax.set_title('Closed-loop damping vs controller call period', loc='left', color=INK, fontsize=11)
    ax.legend(frameon=False, loc='upper right')
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'damping_vs_dt.png'), dpi=150); plt.close(fig)


def fig_timeline():
    sc = batch.get_case('B')
    fig, axs = plt.subplots(2, 1, figsize=(6.8, 5.0), sharex=True)
    for lab, ovr, col in [('v3', {}, C_V3), ('v4', V4, C_V4)]:
        R = gsim.simulate(sc, gsim.params(**ovr), log=True); L = R['log']
        axs[0].plot(L['t'], L['th'] / gsim.D2R, color=col, label=lab + ' turret')
        axs[1].semilogy(L['t'], np.maximum(L['miss'], 1e-2), color=col, lw=1.2, label=lab)
        for s in R['shots']:
            axs[1].plot(s[0], max(s[1], 1e-2), 'o', ms=8, color=col, mec='#fcfcfb', mew=2)
    axs[0].plot(L['t'], L['th_true'] / gsim.D2R, color=MUTED, lw=1.2, ls='--', label='target LOS')
    axs[0].set_ylabel('elevation [deg]'); axs[0].legend(frameon=False, ncol=3, loc='lower right')
    axs[0].set_title('Scenario B: turret angle and beam-to-target distance (dots = pulses)', loc='left', color=INK, fontsize=11)
    axs[1].axhline(2.0, color=MUTED, lw=1, ls=':'); axs[1].text(0.1, 2.4, 'R_eng = 2 m', ha='left', color=MUTED, fontsize=8)
    axs[1].set_ylim(0.01, 1000); axs[1].set_ylabel('beam–target distance [m]'); axs[1].set_xlabel('t [s]')
    axs[1].legend(frameon=False, loc='upper right', ncol=2)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'scenario_B_v3_vs_v4.png'), dpi=150); plt.close(fig)


def fig_summary():
    """bars from round3 results (v3 vs C1n3) across test conditions"""
    rows = [json.loads(l) for l in open(os.path.join(os.path.dirname(OUT), 'results.jsonl')) if '"round3"' in l]
    tests = []
    for r in rows:
        if r['test'] not in tests:
            tests.append(r['test'])
    get = lambda c, t, k: ([r[k] for r in rows if r['cand'] == c and r['test'] == t] or [np.nan])[-1]
    metrics = [('t1_mean', 'mean first-hit time [s]'), ('shots_per', 'pulses per engagement'), ('miss_all', 'mean pulse miss [m]'),
               ('success', 'targets hit [%]')]
    fig, axs = plt.subplots(1, 4, figsize=(13, 0.42 * len(tests) + 1.4), sharey=True)
    y = np.arange(len(tests))
    for ax, (k, lab) in zip(axs, metrics):
        a = [get('v3', t, k) for t in tests]; b = [get('C1n3', t, k) for t in tests]
        ax.barh(y - 0.2, a, 0.38, color=C_V3, label='v3'); ax.barh(y + 0.2, b, 0.38, color=C_V4, label='v4')
        ax.set_title(lab, loc='left', fontsize=10, color=INK); ax.grid(axis='y', visible=False)
        if k == 'success':
            ax.set_xlim(50, 101)
    axs[0].set_yticks(y); axs[0].set_yticklabels([t.replace('H-ho ', '') for t in tests]); axs[0].invert_yaxis()
    axs[0].legend(frameon=False, loc='lower right')
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'robustness_v3_vs_v4.png'), dpi=150); plt.close(fig)


if __name__ == '__main__':
    fig_damping(); fig_timeline(); fig_summary(); print('ok')
