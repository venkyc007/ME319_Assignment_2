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
        h = min(dt, 0.01); nsub = max(1, round(dt/h)); h = dt/nsub;
        for ksub = 1:nsub
            a_r = thdd_ref + 2*P.ref_wf*(thd_ref - S.w_r) + P.ref_wf^2*(th_ref - S.th_r);
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
V_m = min(max(u_fb + u_ff, -P.V_max), P.V_max);

% ---------------- 5. Covariance-gated fire decision ---------------------
r_hat = hypot(S.x(1), S.x(2));
g = [-sin(theta_turret); cos(theta_turret)];        % unit normal to the beam
miss_est = abs(g.'*S.x(1:2));                       % perpendicular distance to beam
sig_perp = sqrt(max(g.'*S.Pk(1:2,1:2)*g, 0));
in_front = [cos(theta_turret) sin(theta_turret)]*S.x(1:2) > 0;
ready = (t - S.last_fire_t) >= P.t_recharge + 1e-6;
m_s = g.'*S.x(1:2);                                 % signed perpendicular offset
want  = converged && in_front && ready && (miss_est + P.fire_k*sig_perp <= P.R_eng);
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
d = struct('newness_by_value', true, 'cross_fire', false, 'ref_wf', 0, 'ref_amax', 20);
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

function [Ad, Bd] = zoh(A, B, dt)
n = size(A,1);
M = expm([A B; zeros(1,n+1)]*dt);
Ad = M(1:n,1:n); Bd = M(1:n,n+1);
end
