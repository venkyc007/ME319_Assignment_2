function C = design_controller(P)
%DESIGN_CONTROLLER  Builds plant Gp(s) and compensator Gc(s) as polynomials.
%   No toolbox needed (pure polyval / roots), so turret_controller.m runs in
%   bare MATLAB or Octave. analysis/plant_and_margins.m does the full
%   Bode / root-locus analysis with the Control toolbox.

% ---- Plant: theta/V = N Kt / ( s [ (J s + b)(L s + R) + N^2 Kt Ke ] ) ----
numP = P.N*P.Kt;
denP = conv([1 0], conv([P.J_eq P.b],[P.Lm P.Rm]) + [0 0 P.N^2*P.Kt*P.Ke]);
C.numP = numP; C.denP = denP;

% lightly damped electromechanical pole pair
q  = conv([P.J_eq P.b],[P.Lm P.Rm]) + [0 0 P.N^2*P.Kt*P.Ke];   % 2nd-order factor
q  = q / q(1);                       % s^2 + 2 z wn s + wn^2
wn = sqrt(q(3)); zp = q(2)/(2*wn);
C.wn = wn; C.zeta_p = zp;

% ---- Compensator pieces ----
notch_num = [1 2*zp*wn wn^2];
notch_den = [1 2*P.zeta_notch_den*wn wn^2];
pi_num = [1 P.z_i];  pi_den = [1 0];
% Lead sized analytically from the target phase margin (instead of a fixed
% lead ratio): phi_m = PM* - (180 + angle(G_rest(j wc))), a = (1+sin)/(1-sin).
% If the notch + PI + plant already supply PM*, no lead is used (a = 1).
wm = P.wc; jw = 1j*wm;
ph = @(c) angle(polyval(c, jw));                 % each factor's angle in (-pi, pi]
phi_rest = (ph(notch_num) - ph(notch_den) + ph(pi_num) - pi/2 ...
           + ph(numP) - ph([1 0]) - ph(q))*180/pi;   % deg, factors summed (no wrap)
phi_m = P.PM_target - (180 + phi_rest);
C.phi_rest = phi_rest; C.phi_lead = max(phi_m, 0);
if phi_m > 0
    if phi_m >= 75, error('design_controller: required lead %.1f deg too large', phi_m); end
    a = (1 + sind(phi_m))/(1 - sind(phi_m));
else
    a = 1;
end
if isfield(P,'lead_a') && ~isempty(P.lead_a), a = P.lead_a; end   % manual override only
C.lead_a = a;
lead_num = [1/(wm/sqrt(a)) 1];  lead_den = [1/(wm*sqrt(a)) 1];

numC0 = conv(conv(notch_num, pi_num), lead_num);
denC0 = conv(conv(notch_den, pi_den), lead_den);

% gain for |L(j wc)| = 1
Lmag = abs(polyval(numC0,jw)/polyval(denC0,jw) * polyval(numP,jw)/polyval(denP,jw));
K = 1/Lmag;
C.K = K;
C.numC = K*numC0;  C.denC = denC0;

% ---- State-space (controllable canonical) for implementation ----
[C.A, C.B, C.Cc, C.D] = local_tf2ss(C.numC, C.denC);

% ---- Inverse-plant feedforward coefficients (low-frequency part) ----
% V = [ (J R + b L) th_ddot + (b R + N^2 Kt Ke) th_dot ] / (N Kt)   (jerk term dropped)
C.ff_v = (P.b*P.Rm + P.N^2*P.Kt*P.Ke)/(P.N*P.Kt);
C.ff_a = (P.J_eq*P.Rm + P.b*P.Lm)/(P.N*P.Kt);
end

function [A,B,Cc,D] = local_tf2ss(num, den)
den = den(:).'; num = num(:).';
num = num/den(1); den = den/den(1);
n = numel(den)-1;
num = [zeros(1, n+1-numel(num)) num];
D = num(1);
r = num(2:end) - D*den(2:end);          % strictly proper remainder
A = [zeros(n-1,1) eye(n-1); -fliplr(den(2:end))];
B = [zeros(n-1,1); 1];
Cc = fliplr(r);
end
