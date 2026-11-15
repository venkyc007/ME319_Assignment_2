% Bode of Gp and loop gain L = Gc Gp: original (lead a=8) vs PM-sized lead.
pkg load control; if isempty(getenv('DISPLAY')), graphics_toolkit('gnuplot'); end
global GUARDIAN_OVERRIDE
w = logspace(-1, 3, 2000);
cfg = {struct('lead_a',8), struct()}; names = {'fixed lead a = 8 (v1 rule)','lead sized for PM^* = 55^o (current)'}; col = {'r','b'};
P0 = guardian_params(); C0 = design_controller(P0); Gp = tf(C0.numP, C0.denP);
f = figure('visible',figvis(),'position',[0 0 900 700]);
[mg, ph] = bode(Gp, w); mg = squeeze(mg); ph = squeeze(ph);
subplot(2,1,1); semilogx(w, 20*log10(mg), 'k--'); hold on;
subplot(2,1,2); semilogx(w, ph, 'k--'); hold on;
for k = 1:2
    GUARDIAN_OVERRIDE = cfg{k}; P = guardian_params(); GUARDIAN_OVERRIDE = [];
    C = design_controller(P); L = tf(C.numC, C.denC)*Gp;
    [mg, ph] = bode(L, w); mg = squeeze(mg); ph = squeeze(ph);
    [gm, pm, wg, wc] = margin(L);
    subplot(2,1,1); semilogx(w, 20*log10(mg), col{k});
    subplot(2,1,2); semilogx(w, ph, col{k});
    fprintf('%s: GM=%.1f dB PM=%.1f deg wc=%.1f\n', names{k}, 20*log10(gm), pm, wc);
end
subplot(2,1,1); grid on; ylabel('|.| [dB]'); ylim([-120 60]); plot(w, 0*w, 'k:');
legend(['G_p'; names(:)], 'location', 'southwest'); title('Open-loop Bode: plant and loop gain L = G_cG_p');
subplot(2,1,2); grid on; ylabel('phase [deg]'); xlabel('\omega [rad/s]'); ylim([-300 0]); plot(w, -180+0*w, 'k:');
print(f, 'figs/bode_loop.png', '-dpng', '-r110');
