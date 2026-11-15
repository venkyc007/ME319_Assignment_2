function M = margins_check(ovr, quiet)
global GUARDIAN_OVERRIDE
if nargin<1, ovr=struct(); end
if nargin<2, quiet=false; end
pkg load control
GUARDIAN_OVERRIDE = ovr; P = guardian_params(); GUARDIAN_OVERRIDE = [];
C = design_controller(P);
Gp = tf(C.numP, C.denP); Gc = tf(C.numC, C.denC); L = Gc*Gp;
[gm,pm,wg,wc] = margin(L);
S = feedback(1,L); w=logspace(-1,3,3000); Ms = max(abs(squeeze(freqresp(S,w))));
T = feedback(L,1); p = pole(T);
M = struct('K',C.K,'a',C.lead_a,'GM',20*log10(gm),'wg',wg,'PM',pm,'wc',wc,'Ms',Ms,'D',C.D,'maxRe',max(real(p)),'phi_rest',C.phi_rest);
if ~quiet
fprintf('wc*=%g PM*=%g: K=%.0f a=%.2f (rest phase %.1f) GM=%.1f dB @%.1f  PM=%.1f deg @%.1f  Ms=%.2f  HF gain=%.0f V/rad maxRe(CL)=%.2f\n', ...
  P.wc, P.PM_target, C.K, C.lead_a, C.phi_rest, M.GM, wg, pm, wc, Ms, C.D, M.maxRe);
end
end
