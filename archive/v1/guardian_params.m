function P = guardian_params()
%GUARDIAN_PARAMS  Single source of truth for all physical and design parameters.
%   Physical values are from the ME319 Assignment 2 brief (Section 2).
%   Design values (marked DESIGN) are our choices and must be justified in the paper.

% ---------------- Physical plant (brief, Sec. 2) ----------------
P.J_turret = 1200;      % kg m^2, turret + cradle about trunnion
P.Jm       = 0.20;      % kg m^2, motor rotor
P.N        = 100;       % gear ratio (motor speed = N * turret speed)
P.b        = 1500;      % N m s/rad, turret-side viscous damping
P.Kt       = 10;        % N m/A
P.Ke       = 10;        % V s/rad
P.Rm       = 1.0;       % ohm
P.Lm       = 0.10;      % H
P.J_eq     = P.J_turret + P.N^2 * P.Jm;   % reflected inertia at turret = 3200 kg m^2
P.theta_min = -10*pi/180;
P.theta_max =  85*pi/180;

% ---------------- Sensor / engagement (brief) ----------------
P.Ts_radar  = 0.05;          % s, 20 Hz
P.sig_r     = 1.0;           % m
P.sig_th    = 0.02*pi/180;   % rad
P.tp        = 0.50;          % s, prescribed prediction horizon
P.R_eng     = 2.0;           % m
P.t_recharge = 1.0;          % s
P.t_end     = 10.0;          % s

% ---------------- Target envelope (brief, Sec. 3) ----------------
P.r_min = 800; P.r_max = 2000; P.v_max = 150; P.a_max = 20;

% ---------------- DESIGN: estimator ----------------
P.q_jerk   = 20^2;   % DESIGN: jerk PSD [m^2/s^5], CA model (tuned on R1-R100, Nov 6)
P.n_init   = 6;      % DESIGN: measurements before estimate is trusted for firing

% ---------------- DESIGN: reference / fire control ----------------
P.mode      = 'track';   % DESIGN: 'track' (continuous) or 'ambush' (aim-point & wait)
P.tau_ref   = 0.0;       % DESIGN: extra lead used in 'track' mode [s]
P.fire_k    = 3.0;       % DESIGN: fire if miss_est + fire_k*sigma_perp <= R_eng (3-sigma; tuned R1-R100)
P.use_ff    = true;      % DESIGN: inverse-plant velocity/acceleration feedforward

% ---------------- DESIGN: feedback compensator Gc(s) ----------------
% Gc(s) = K * notch(s) * (s+z_i)/s * lead(s)
P.wc     = 20;      % rad/s target crossover
P.z_i    = 2;       % rad/s PI zero
P.PM_target = 55;   % DESIGN: target phase margin [deg]; lead ratio derived from it
P.lead_a = [];      % (empty = derive from PM_target; set only for design studies)
P.zeta_notch_den = 0.7;  % damping of replacement poles in the notch
P.V_max  = inf;     % motor voltage saturation [V] (none specified in brief)

% ---------------- optional overrides (for design studies only) ----------------
global GUARDIAN_OVERRIDE
if isstruct(GUARDIAN_OVERRIDE)
    fn = fieldnames(GUARDIAN_OVERRIDE);
    for k = 1:numel(fn), P.(fn{k}) = GUARDIAN_OVERRIDE.(fn{k}); end
end
end
