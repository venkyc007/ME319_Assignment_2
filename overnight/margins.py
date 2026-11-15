import numpy as np, gsim, math
def margins(P):
    C = gsim.design(P)
    w = np.logspace(-1, 3.5, 200000); s = 1j*w
    L = np.polyval(C['numC'], s)/np.polyval(C['denC'], s)*np.polyval(C['numP'], s)/np.polyval(C['denP'], s)
    mag = np.abs(L); ph = np.unwrap(np.angle(L))*180/math.pi
    ic = np.where(np.diff(np.sign(mag-1)))[0]; wc = w[ic[0]]; pm = 180+ph[ic[0]]
    ip = np.where(np.diff(np.sign(ph+180)))[0]
    gm = -20*np.log10(mag[ip[0]]) if len(ip) else float('inf'); wpc = w[ip[0]] if len(ip) else float('nan')
    Ms = np.max(1/np.abs(1+L))
    T = L/(1+L); bw = w[np.where(np.abs(T) < 1/math.sqrt(2))[0][0]]
    return dict(wc=wc, PM=pm, GM_dB=gm, w180=wpc, Ms=Ms, bw=bw, lead_a=C['lead_a'], K=C['K'])
if __name__ == '__main__':
    for lab, P in [('v3 notch-PI-lead wc10', gsim.params()),
                   ('damped Rv5 PI-lead wc15', gsim.params(servo='damped', Rv=5.0, wc=15.0)),
                   ('damped Rv1 PI-lead wc15 (50ms)', gsim.params(servo='damped', Rv=1.0, wc=15.0))]:
        m = margins(P); print(lab, {k: round(v, 2) for k, v in m.items()})
    # robustness: plant J,L,Kt +-20% with the nominal controller (continuous)
