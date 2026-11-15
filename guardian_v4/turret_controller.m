function [V_m, fire_cmd, dbg] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
%TURRET_CONTROLLER  Guardian predictive-feedback fire control (elevation axis).
%
%   Radar (r,theta) -> EKF, constant-acceleration target model (Cartesian)
%                   -> CA prediction to t + tp
%                   -> predicted elevation reference theta_ref (+ derivatives)
%                   -> LTI compensator Gc(s) (+ inverse-plant feedforward) -> V_m
%   Fire logic: covariance-gated -- fire when the estimated perpendicular miss
%   of the beam, plus fire_k standard deviations, is inside R_eng.
%
%   State is kept in persistent variables; it resets automatically when t
%   goes backwards or t == 0 (new run). 3rd output dbg is for logging only.

persistent P C S
if isempty(S) || (t <= 0 && S.last_t > 0) || t < S.last_t - 0.5
    P = guardian_params();
    P = defaults2(P);
    C = design_controller(P);
    S = init_state(P, C);
end
if t < S.last_t - 1e-9          % small backward step (e.g. ODE solver trial): no state change
    V_m = S.V_last; fire_cmd = 0; dbg = S.dbg_last; return;
end
dt = max(t - S.last_t, 0);
S.last_t = t;

% ---------------- 0. (v4) armature-current observer + Rv schedule --------
% The voltage applied over the last interval (S.V_last) is known exactly, and
% theta_dot is measured, so the current is reconstructed in one step from the
% ZOH-discretised plant model:  w(k) = Phi22 w(k-1) + Phi23 i(k-1) + Gam2 V(k-1).
if dt > 0 && strcmp(P.servo, 'damped')
    if P.Rv_sched && abs(dt - S.dt_design) > 0.2*dt
        % largest Rv that keeps the sampled loop well damped (discrete-time analysis, REPORT sec. 4)
        S.dt_design = dt;
        if dt > 0.03
            % slow evaluator (call period > 30 ms): the sampled current loop cannot be
            % made well damped, so fall back to the v3 servo (notch-PI-lead, wc 10) + linear shaper
            P.servo = 'notch'; P.Rv = 0; P.wc = 10; P.shaper = 'linear';
        elseif dt <= 0.021
            P.Rv = 5;
        else
            P.Rv = 3;
        end
        C = design_controller(P); S.dt_cached = -1;
        if numel(S.xc) ~= size(C.A,1), S.xc = zeros(size(C.A,1),1); end
    end
end
if dt > 0 && strcmp(P.servo, 'damped')
    if abs(dt - S.dt_obs) > 1e-12
        [S.Phi, S.Gam] = plant_zoh(P, dt); S.dt_obs = dt;
    end
    i_prev  = (theta_dot_turret - S.Phi(2,2)*S.thd_prev - S.Gam(2)*S.V_last) / S.Phi(2,3);
    S.i_hat = S.Phi(3,2)*S.thd_prev + S.Phi(3,3)*i_prev + S.Gam(3)*S.V_last;
end
S.thd_prev = theta_dot_turret;

% ---------------- 1. Estimator: propagate to t ----------------
if S.initialised && dt > 0
    [S.x, S.Pk] = ca_predict(S.x, S.Pk, dt, P.q_jerk);
end

% ---------------- 2. Measurement update (new radar sample only) -----------
ok_meas = meas_valid && isfinite(r_meas) && isfinite(theta_meas);
if P.newness_by_value
    is_new = ok_meas && ~(r_meas == S.z_last(1) && theta_meas == S.z_last(2));
else
    is_new = ok_meas && (t - S.t_last_meas) > 0.5*P.Ts_radar;
end
if is_new, S.z_last = [r_meas; theta_meas]; end
if is_new
    S.t_last_meas = t;
    z = [r_meas; theta_meas];
    Rz = diag([P.sig_r^2, P.sig_th^2]);
    if ~S.initialised
        S.x = [r_meas*cos(theta_meas); r_meas*sin(theta_meas); 0; 0; 0; 0];
        Jp = [cos(theta_meas) -r_meas*sin(theta_meas); sin(theta_meas) r_meas*cos(theta_meas)];
        S.Pk = blkdiag(Jp*Rz*Jp.', (P.v_max)^2*eye(2), (P.a_max)^2*eye(2));
        S.initialised = true;
    else
        [S.x, S.Pk] = ekf_update(S.x, S.Pk, z, Rz);
    end
    S.n_meas = S.n_meas + 1;
end

% ---------------- 3. Prediction + reference generation ------------------
V_m = 0; fire_cmd = 0;
dbg = struct('x',nan(6,1),'th_ref',nan,'th_hat',nan,'miss_est',nan,'sig_perp',nan,'xp',nan(6,1),'converged',false,'in_front',false);
if ~S.initialised
    S.fire_prev = 0; return;
end
converged = S.n_meas >= P.n_init;

xp = ca_extrap(S.x, P.tp);                 % predicted state at t + tp
[th_hat, ~, ~] = los_angle(S.x);           % current estimated elevation

mode = P.mode;
if strcmp(mode,'hybrid'), if S.n_fired == 0, mode = 'ambush'; else, mode = 'track'; end, end
switch mode
    case 'ambush'
        % Aim-point fire control: slew to the elevation the target is predicted
        % to occupy tp seconds ahead, hold, fire as it crosses the beam.
        if ~converged
            [th_ref, thd_ref, thdd_ref] = los_angle(S.x);
        else
            if isnan(S.aim_t) || t > S.aim_t + 0.25 || S.fired_on_aim
                S.aim_t = t + P.tp;
                S.aim_th = los_angle(xp);
                S.fired_on_aim = false;
            end
            th_ref = S.aim_th; thd_ref = 0; thdd_ref = 0;
        end
    case 'ambush2'
        % v4 aim-point fire control. The aim time T is placed tp ahead (T = t + tp);
        % on every later call the aim is re-predicted to the SAME absolute time T
        % with the newest estimate (shrinking horizon T - t), so the beam is on the
        % target's predicted position exactly at a controller call. After a pulse
        % the next aim is placed only when t + tp reaches recharge completion,
        % giving one engagement opportunity per recharge period.
        if ~converged
            [th_ref, thd_ref, thdd_ref] = los_angle(S.x);
        else
            t_ready = S.last_fire_t + P.t_recharge;
            if S.fired_on_aim
                if t + P.tp >= t_ready - 1e-9
                    S.aim_t = t + P.tp; S.fired_on_aim = false;
                end
            elseif isnan(S.aim_t) || t > S.aim_t + P.grace
                S.aim_t = t + P.tp;
            end
            if ~S.fired_on_aim
                S.aim_th = los_angle(ca_extrap(S.x, max(S.aim_t - t, 0)));
            end
            th_ref = S.aim_th; thd_ref = 0; thdd_ref = 0;
        end
    otherwise   % 'track'
        [th_ref, thd_ref, thdd_ref] = los_angle(ca_extrap(S.x, P.tau_ref));
end
% respect mechanical limits with 1 deg margin
th_ref = min(max(th_ref, P.theta_min + pi/180), P.theta_max - pi/180);
% ---- optional 2-DOF reference shaper (critically damped, accel-limited):
%   theta_r'' = sat( theta_d'' + 2 wf (theta_d' - w_r) + wf^2 (theta_d - theta_r), +-a_max )
%   Error dynamics are LTI (e'' + 2wf e' + wf^2 e = 0) when unsaturated, so it
%   adds no steady tracking lag; it only shapes the initial acquisition / re-aims.
if P.ref_wf > 0
    if ~S.ref_init, S.th_r = theta_turret; S.w_r = theta_dot_turret; S.ref_init = true; end
    if dt > 0
        if strcmp(P.shaper,'ptos'), hmax = 0.002; else, hmax = 0.01; end
        h = min(dt, hmax); nsub = max(1, round(dt/h)); h = dt/nsub;
        for ksub = 1:nsub
            if strcmp(P.shaper,'ptos')
                % proximate time-optimal: brake at alpha*amax along sqrt profile,
                % linear zone of gain k near the target (C1 junction at e_l = alpha*amax/k^2)
                ee = th_ref - S.th_r; ae = abs(ee);
                aa = P.ptos_alpha*P.ref_amax; el = aa/P.ptos_k^2;
                if ae > el, fv = sqrt(2*aa*(ae - el/2)); else, fv = P.ptos_k*ae; end
                v_des = thd_ref + sign(ee)*fv;
                a_r = thdd_ref + P.ptos_kv*(v_des - S.w_r);
            else
                a_r = thdd_ref + 2*P.ref_wf*(thd_ref - S.w_r) + P.ref_wf^2*(th_ref - S.th_r);
            end
            a_r = min(max(a_r, -P.ref_amax), P.ref_amax);
            S.th_r = S.th_r + h*S.w_r + 0.5*h^2*a_r; S.w_r = S.w_r + h*a_r;
        end
    else
        a_r = 0;
    end
    th_ref = S.th_r; thd_ref = S.w_r; thdd_ref = a_r;
end

% ---------------- 4. Feedback compensator Gc(s) + feedforward ------------
e = th_ref - theta_turret;
if dt > 0
    if abs(dt - S.dt_cached) > 1e-12
        [S.Ad, S.Bd] = zoh(C.A, C.B, dt); S.dt_cached = dt;
    end
    S.xc = S.Ad*S.xc + S.Bd*S.e_prev;      % ZOH-discretised compensator
end
u_fb = C.Cc*S.xc + C.D*e;
S.e_prev = e;
u_ff = 0;
if P.use_ff
    u_ff = C.ff_v*thd_ref + C.ff_a*thdd_ref;
end
u = u_fb + u_ff;
if strcmp(P.servo,'damped'), u = u - P.Rv*S.i_hat; end   % virtual resistance (inner current loop)
V_m = min(max(u, -P.V_max), P.V_max);

% ---------------- 5. Covariance-gated fire decision ---------------------
r_hat = hypot(S.x(1), S.x(2));
g = [-sin(theta_turret); cos(theta_turret)];        % unit normal to the beam
miss_est = abs(g.'*S.x(1:2));                       % perpendicular distance to beam
sig_perp = sqrt(max(g.'*S.Pk(1:2,1:2)*g, 0));
in_front = [cos(theta_turret) sin(theta_turret)]*S.x(1:2) > 0;
ready = (t - S.last_fire_t) >= P.t_recharge + 1e-6;
m_s = g.'*S.x(1:2);                                 % signed perpendicular offset
want  = converged && in_front && ready && (miss_est + P.fire_k*sig_perp <= P.R_eng);
if P.fire_best && want && dt > 0
    % v4: if the target will be clearly closer to the beam at the next call
    % (assumed one call period later) and still inside the gate, wait for it.
    md = g.'*S.x(3:4) - theta_dot_turret*([cos(theta_turret) sin(theta_turret)]*S.x(1:2));
    m_next = m_s + md*dt;
    if abs(m_next) < miss_est - 0.05 && abs(m_next) + P.fire_k*sig_perp <= P.R_eng
        want = false;
    end
end
if P.cross_fire && ~want && converged && in_front && ready && dt > 0
    % beam-normal rate of the target relative to the (rotating) beam
    md = g.'*S.x(3:4) - theta_dot_turret*([cos(theta_turret) sin(theta_turret)]*S.x(1:2));
    m_next = m_s + md*dt;                            % expected offset at next call
    best_now = abs(m_s) <= abs(m_next) && sign(m_next) ~= sign(m_s);  % crossing before next call, now is closer
    best_now = best_now || (abs(m_s) <= abs(m_next) && abs(S.m_prev) > abs(m_s) && sign(S.m_prev) == sign(m_s)); % local minimum
    want = best_now && (abs(m_s) + P.fire_k*sig_perp <= P.R_eng);
end
S.m_prev = m_s;
if want && S.fire_prev == 0
    fire_cmd = 1; S.last_fire_t = t; S.fired_on_aim = true; S.n_fired = S.n_fired + 1;
end
S.fire_prev = fire_cmd;
S.V_last = V_m;

dbg.x = S.x; dbg.xp = xp; dbg.th_ref = th_ref; dbg.th_hat = th_hat;
dbg.miss_est = miss_est; dbg.sig_perp = sig_perp; dbg.r_hat = r_hat;
dbg.converged = converged; dbg.in_front = in_front;
S.dbg_last = dbg;
end

function P = defaults2(P)
d = struct('newness_by_value', true, 'cross_fire', false, 'ref_wf', 0, 'ref_amax', 20, ...
           'grace', 0.10, 'fire_best', false, 'shaper', 'linear', 'ptos_alpha', 0.8, ...
           'ptos_k', 15, 'ptos_kv', 60, 'servo', 'notch', 'Rv', 0, 'Rv_sched', false);
f = fieldnames(d);
for k = 1:numel(f), if ~isfield(P, f{k}), P.(f{k}) = d.(f{k}); end, end
end

% ======================================================================
function S = init_state(P, C)
S.last_t = 0; S.initialised = false; S.n_meas = 0; S.t_last_meas = -inf;
S.x = zeros(6,1); S.Pk = eye(6);
S.xc = zeros(size(C.A,1),1); S.e_prev = 0; S.dt_cached = -1; S.Ad = []; S.Bd = [];
S.last_fire_t = -inf; S.fire_prev = 0;
S.aim_t = NaN; S.aim_th = NaN; S.fired_on_aim = false;
S.z_last = [NaN; NaN]; S.n_fired = 0; S.ref_init = false; S.th_r = 0; S.w_r = 0; S.m_prev = NaN; S.V_last = 0;
S.i_hat = 0; S.thd_prev = 0; S.dt_obs = -1; S.dt_design = -1; S.Phi = eye(3); S.Gam = zeros(3,1);
S.dbg_last = struct('x',nan(6,1),'th_ref',nan,'th_hat',nan,'miss_est',nan,'sig_perp',nan,'xp',nan(6,1),'converged',false,'in_front',false);
end

function F = ca_F(dt)
I = eye(2); Z = zeros(2);
F = [I dt*I dt^2/2*I; Z I dt*I; Z Z I];
end

function [x, Pk] = ca_predict(x, Pk, dt, q)
F = ca_F(dt);
Q1 = q*[dt^5/20 dt^4/8 dt^3/6; dt^4/8 dt^3/3 dt^2/2; dt^3/6 dt^2/2 dt];
Q = kron(Q1, eye(2));                     % state order [x y vx vy ax ay]
x = F*x; Pk = F*Pk*F.' + Q;
end

function xp = ca_extrap(x, tau)
xp = ca_F(tau)*x;
end

function [x, Pk] = ekf_update(x, Pk, z, Rz)
px = x(1); py = x(2); r = hypot(px,py);
h = [r; atan2(py,px)];
H = [px/r py/r 0 0 0 0; -py/r^2 px/r^2 0 0 0 0];
y = z - h; y(2) = atan2(sin(y(2)), cos(y(2)));
Sm = H*Pk*H.' + Rz;
K = Pk*H.'/Sm;
x = x + K*y;
IKH = eye(6) - K*H;
Pk = IKH*Pk*IKH.' + K*Rz*K.';             % Joseph form
end

function [th, thd, thdd] = los_angle(x)
px = x(1); py = x(2); vx = x(3); vy = x(4); ax = x(5); ay = x(6);
r2 = px^2 + py^2;
th = atan2(py, px);
c = px*vy - py*vx;
thd = c/r2;
thdd = (px*ay - py*ax)/r2 - 2*(px*vx + py*vy)*c/r2^2;
end

function [Phi, Gam] = plant_zoh(P, dt)
% ZOH discretisation of the armature/mechanical model, state [theta; theta_dot; i]
J = P.J_eq;
A = [0 1 0; 0 -P.b/J P.N*P.Kt/J; 0 -P.Ke*P.N/P.Lm -P.Rm/P.Lm];
B = [0; 0; 1/P.Lm];
[Phi, Gam] = zoh(A, B, dt);
end

function [Ad, Bd] = zoh(A, B, dt)
n = size(A,1);
M = expm([A B; zeros(1,n+1)]*dt);
Ad = M(1:n,1:n); Bd = M(1:n,n+1);
end
