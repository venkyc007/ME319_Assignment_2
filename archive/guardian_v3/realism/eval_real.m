function R = eval_real(C, o)
%EVAL_REAL  Stress harness with REALISTIC non-idealities layered on top of the
%   assignment model. The controller under test is NOT changed or retuned:
%   it still believes the assigned parameters (sig_th = 0.02 deg, 20 Hz, ideal drive).
%
%   Sensor options
%     .sig_th   radar angle noise std [rad]           (assigned 0.02 deg)
%     .sig_r    radar range noise std [m]             (assigned 1 m)
%     .glint_p  probability a sample is a glint/multipath outlier (0)
%     .glint_k  outlier size as multiple of sig_th (10)
%     .Ts_r     radar update period [s]              (assigned 0.05)
%     .lat      measurement latency [s]: sample delivered lat late (0)
%     .bo_scale multiply every blackout duration (1)  (>1 goes beyond spec)
%   Actuator / mechanics options
%     .Vmax     drive voltage saturation [V]         (inf)
%     .tau_c    Coulomb friction at turret [N m]      (0)
%     .tau_g    gravity-unbalance torque amplitude [N m] (x cos(theta)) (0)
%     .backlash total gear backlash at turret [rad]   (0)
%     .enc_q    angle-encoder quantum [rad]          (0)
%     .rate_sig gyro/tacho rate noise std [rad/s]     (0)
%     .scale    [J L b Kt] plant-parameter multipliers ([1 1 1 1])
%   Beam option
%     .jit      beam pointing jitter std at shot [rad] (0) (structure + atmosphere)
%   Timing options as eval_stress: .dt, .meas, .th0, .seed, .ovr, .ctrl
global GUARDIAN_OVERRIDE
if nargin < 2, o = struct(); end
d = struct('dt',1e-3,'meas','pulse','th0',0,'seed',C.id,'ovr',struct(),'ctrl','turret_controller', ...
    'sig_th',0.02*pi/180,'sig_r',1,'glint_p',0,'glint_k',10,'Ts_r',0.05,'lat',0,'bo_scale',1, ...
    'Vmax',inf,'tau_c',0,'tau_g',0,'backlash',0,'enc_q',0,'rate_sig',0,'scale',[1 1 1 1],'jit',0);
fn = fieldnames(d); for k = 1:numel(fn), if ~isfield(o,fn{k}), o.(fn{k}) = d.(fn{k}); end, end
GUARDIAN_OVERRIDE = o.ovr;
clear turret_controller turret_controller2 turret_plant
P = guardian_params();
ctrl = str2func(o.ctrl);
randn('state', 31337 + o.seed); rand('state', 4242 + o.seed);
Ts = o.Ts_r; nK = ceil(P.t_end/Ts) + 2;
nr = randn(nK,1)*o.sig_r; nth = randn(nK,1)*o.sig_th;
gl = rand(nK,1) < o.glint_p; nth(gl) = nth(gl) + o.glint_k*o.sig_th*sign(randn(sum(gl),1)).*(0.5+rand(sum(gl),1));
rate_n = randn(1e5,1)*o.rate_sig; jit_n = randn(100,1)*o.jit;
bo = C.bo; bo(:,2) = bo(:,2)*o.bo_scale;
tv = (0:o.dt:P.t_end).'; n = numel(tv);
x = [o.th0; 0; 0]; th_b = o.th0;               % motor-side angle, rate, current; beam (load) angle
inbo = @(tt) any(tt >= bo(:,1) - 1e-9 & tt < bo(:,1) + bo(:,2) - 1e-9);
gdt = C.tg(2) - C.tg(1);
truth = @(tt) C.X(min(max(floor(tt/gdt + 1e-9),0), size(C.X,1)-1) + 1, :);
k_last = -1; rm = NaN; thm = NaN; have = false;
shots = zeros(0,3); fire_prev = 0; last_pulse = -inf; nshot = 0;
perr = zeros(n,1); Vabs = zeros(n,1);
for k = 1:n
    t = tv(k);
    kk = floor(t/Ts + 1e-9); newsamp = false;
    if kk > k_last
        k_last = kk;
        ts = kk*Ts - o.lat;                     % measurement describes the target at ts
        if ~inbo(kk*Ts) && ts >= 0
            Xk = truth(ts);
            rm = hypot(Xk(1),Xk(2)) + nr(kk+1); thm = atan2(Xk(2),Xk(1)) + nth(kk+1);
            newsamp = true; have = true;
        end
    end
    switch o.meas
        case 'pulse', mv = double(newsamp);
        case 'hold',  mv = double(have && ~inbo(t));
    end
    r_in = rm; th_in = thm;
    if ~have, r_in = NaN; th_in = NaN; mv = 0; end
    th_meas = x(1); if o.enc_q > 0, th_meas = o.enc_q*round(x(1)/o.enc_q); end
    w_meas = x(2) + rate_n(mod(k-1,numel(rate_n))+1);
    [V, fc] = ctrl(t, r_in, th_in, mv, th_meas, w_meas);
    V = min(max(V, -o.Vmax), o.Vmax);
    X = truth(t);
    if fc == 1 && fire_prev == 0 && t - last_pulse >= P.t_recharge - 1e-9
        last_pulse = t; nshot = nshot + 1;
        dm = ray_miss(th_b + jit_n(min(nshot,100)), X(1:2));
        shots(end+1,:) = [t dm dm <= P.R_eng]; %#ok<AGROW>
    end
    fire_prev = fc;
    perr(k) = ray_miss(th_b, X(1:2)); Vabs(k) = abs(V);
    if k < n
        f = @(xx) plant_real(V, xx, P, o);
        h = o.dt; k1 = f(x); k2 = f(x+h/2*k1); k3 = f(x+h/2*k2); k4 = f(x+h*k3);
        x = x + h/6*(k1+2*k2+2*k3+k4);
        % kinematic backlash: load (beam) angle stays put until the motor side takes up the play
        if o.backlash > 0
            th_b = min(max(th_b, x(1) - o.backlash/2), x(1) + o.backlash/2);
        else
            th_b = x(1);
        end
    end
    if any(~isfinite(x)) || abs(x(1)) > 10, break; end
end
GUARDIAN_OVERRIDE = [];
R.id = C.id; R.fam = C.fam; R.shots = shots; R.n_shots = size(shots,1);
R.n_hits = sum(shots(:,3)); R.success = R.n_hits > 0;
R.t1 = NaN; if R.success, R.t1 = shots(find(shots(:,3),1),1); end
R.hit_miss = mean(shots(shots(:,3)==1,2));
R.first_shot_t = NaN; if R.n_shots, R.first_shot_t = shots(1,1); end
m = tv >= 3 & (1:n).' <= k;
R.rms_perp_t3 = sqrt(mean(perr(m).^2));
R.Vmax_after05 = max(Vabs(tv > 0.5 & (1:n).' <= k));
R.unstable = any(~isfinite(x)) || abs(x(1)) > 10 || R.rms_perp_t3 > 200;
R.miss_all = shots(:,2);
end

function dx = plant_real(V, x, P, o)
s = o.scale;
Jeq = s(1)*P.J_eq; L = s(2)*P.Lm; b = s(3)*P.b; Kt = s(4)*P.Kt; Ke = s(4)*P.Ke;
w = x(2);
tau = P.N*Kt*x(3) - b*w - o.tau_c*tanh(w/2e-3) - o.tau_g*cos(x(1));
if x(1) > P.theta_max, tau = tau - 1e7*(x(1)-P.theta_max) - 1e5*max(w,0);
elseif x(1) < P.theta_min, tau = tau - 1e7*(x(1)-P.theta_min) - 1e5*min(w,0); end
dx = [w; tau/Jeq; (V - P.Rm*x(3) - Ke*P.N*w)/L];
end
function dm = ray_miss(th, p)
u = [cos(th); sin(th)]; s = u.'*p(:);
if s < 0, dm = norm(p); else, dm = abs(-sin(th)*p(1) + cos(th)*p(2)); end
end
