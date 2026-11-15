%PLANT_AND_MARGINS  Analytical support for the paper (needs Control toolbox /
%   Octave 'control' package). Produces: plant poles, Bode of Gp and L=GcGp,
%   margins, closed-loop step and ramp response, P-only gain ceiling.
if exist('OCTAVE_VERSION','builtin'), pkg load control; end
addpath('..');
P = guardian_params(); C = design_controller(P);
s  = tf('s');
Gp = tf(C.numP, C.denP);
Gc = tf(C.numC, C.denC);
Lp = Gc*Gp; T = feedback(Lp,1);

fprintf('J_eq = %.0f kg m^2 (motor share %.0f%%)\n', P.J_eq, 100*P.N^2*P.Jm/P.J_eq);
fprintf('Plant poles:\n'); disp(pole(Gp));
fprintf('wn = %.2f rad/s, zeta = %.4f\n', C.wn, C.zeta_p);
fprintf('Low-freq velocity gain = %.3e rad/s per V\n', P.N*P.Kt/(P.b*P.Rm + P.N^2*P.Kt*P.Ke));
Kmax = 1/abs(evalfr(Gp, 1j*C.wn*sqrt(1-2*C.zeta_p^2)));
fprintf('P-only stability gain ceiling approx K = %.0f V/rad\n', Kmax);
[gm,pm,wg,wc] = margin(Lp);
fprintf('Designed loop: K=%.0f GM=%.1f dB PM=%.1f deg wc=%.1f rad/s\n', C.K, 20*log10(gm), pm, wc);
fprintf('Closed-loop poles:\n'); disp(pole(T));
Ka = dcgain(minreal(s^2*Lp));
fprintf('Acceleration error constant Ka = %.1f 1/s^2\n', Ka);

figure; bode(Gp, Lp); grid on; legend('G_p','G_cG_p'); title('Open-loop Bode');
figure; step(T, 3); grid on; title('Closed-loop step');
figure; t = 0:1e-3:3; lsim(T, 0.15*t, t); grid on; title('Ramp (0.15 rad/s) response');
figure; rlocus(Gp); title('Root locus, P-only');
