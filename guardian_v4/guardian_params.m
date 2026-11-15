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
P.mode      = 'ambush';  % DESIGN (v3): 'ambush' = aim at the elevation predicted at t+tp (Sec. 4 of the brief:
                         %   reference = predicted state at tp), hold, fire as target crosses the beam.
                         %   'track' = follow the CURRENT estimate (tp NOT used for the reference; comparison only).
P.tau_ref   = 0.0;       % DESIGN: lead used in 'track' mode [s]. tau_ref = tp makes the turret run ~25 m ahead (never hits).
P.fire_k    = 3.0;       % DESIGN: fire if miss_est + fire_k*sigma_perp <= R_eng (3-sigma; tuned R1-R100)
P.use_ff    = true;      % DESIGN: inverse-plant velocity/acceleration feedforward

% ---------------- DESIGN: feedback compensator Gc(s) ----------------
% Gc(s) = K * notch(s) * (s+z_i)/s * lead(s)
P.wc     = 10;      % rad/s target crossover (was 20: robust to 50 ms control steps and +-20% J/L/Kt)
P.z_i    = 5;       % rad/s PI zero (was 2: slow -2.2 rad/s closed-loop pole dominated first-hit time)
P.PM_target = 55;   % DESIGN: target phase margin [deg]; lead ratio derived from it
P.lead_a = [];      % (empty = derive from PM_target; set only for design studies)
P.zeta_notch_den = 0.7;  % damping of replacement poles in the notch
P.newness_by_value = true; % DESIGN: new radar sample = valid flag AND changed value (works for pulse or hold conventions)
P.cross_fire = false;
P.ref_wf = 15;      % DESIGN: 2-DOF reference shaper bandwidth [rad/s] (0 = off); removes acquisition overshoot into the 85 deg stop
P.ref_amax = 20;    % rad/s^2, shaper acceleration limit
P.V_max  = inf;     % motor voltage saturation [V] (none specified in brief)

% ---------------- v4 changes (overnight study 15 Nov, see overnight/REPORT) ----------------
P.n_init  = 3;           % v4: 3 radar samples (0.10 s) before firing/aiming (v3: 6)
P.mode    = 'ambush2';   % v4: aim point placed tp ahead, refined to the SAME absolute aim time each call,
                         %     next aim placed when t+tp reaches recharge completion (1 pulse per second)
P.grace   = 0.10;        % v4: re-aim if no pulse within grace [s] after the aim time
P.fire_best = true;      % v4: inside the gate, wait one call if the target will be closer to the beam
P.shaper  = 'ptos';      % v4: proximate-time-optimal slew (v3: 'linear' critically damped)
P.ptos_alpha = 0.8;      %     fraction of ref_amax used for braking
P.ptos_k  = 15;          %     terminal linear-zone gain [rad/s]; zone width = alpha*amax/k^2
P.ptos_kv = 60;          %     velocity-loop gain of the shaper [1/s]
P.servo   = 'damped';    % v4: current feedback (virtual resistance Rv) replaces the notch
P.Rv      = 5;           % ohm, virtual resistance: plant mode zeta 0.094 -> ~0.5
P.Rv_sched = true;       % reduce Rv when the controller is called slowly (discrete stability table)
P.wc      = 15;          % rad/s crossover for the PI-lead on the damped plant (PM 55 deg, GM 11.9 dB)

% ---------------- optional overrides (for design studies only) ----------------
global GUARDIAN_OVERRIDE
if isstruct(GUARDIAN_OVERRIDE)
    fn = fieldnames(GUARDIAN_OVERRIDE);
    for k = 1:numel(fn), P.(fn{k}) = GUARDIAN_OVERRIDE.(fn{k}); end
end
end
