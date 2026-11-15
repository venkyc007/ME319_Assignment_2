function R = eval_stress(C, o)
%EVAL_STRESS  Independent evaluation harness (mimics plausible instructor scripts).
%   C : case from stress_case (or struct with tg, X, bo)
%   o : options
%     .dt       controller/plant step [s]            (default 1e-3)
%     .meas     'pulse' valid=1 only on the sample step (default)
%               'hold'  valid=1 every step, last sample held, 0 in blackout
%               'nan'   like pulse but r/theta = NaN when no new sample
%     .integ    'rk4' | 'euler'                       (default rk4)
%     .th0      initial turret angle [rad]            (default 0)
%     .scale    [J L b Kt] multipliers for plant mismatch (default [1 1 1 1]);
%               if not all ones an inline plant is used instead of turret_plant
%     .seed     noise seed
%     .ovr      struct of guardian_params overrides
global GUARDIAN_OVERRIDE
if nargin < 2, o = struct(); end
d = struct('dt',1e-3,'meas','pulse','integ','rk4','th0',0,'scale',[1 1 1 1],'seed',C.id,'ovr',struct(),'ctrl','turret_controller');
fn = fieldnames(d); for k = 1:numel(fn), if ~isfield(o,fn{k}), o.(fn{k}) = d.(fn{k}); end, end
GUARDIAN_OVERRIDE = o.ovr;
clear turret_controller turret_controller2 turret_plant
P = guardian_params();
ctrl = str2func(o.ctrl);
randn('state', 31337 + o.seed);
Ts = P.Ts_radar; nK = ceil(P.t_end/Ts) + 2;
nr = randn(nK,1)*P.sig_r; nth = randn(nK,1)*P.sig_th;
tv = (0:o.dt:P.t_end).'; n = numel(tv);
x = [o.th0; 0; 0];
inbo = @(tt) any(tt >= C.bo(:,1) - 1e-9 & tt < C.bo(:,1) + C.bo(:,2) - 1e-9);
gdt = C.tg(2) - C.tg(1);
truth = @(tt) C.X(min(floor(tt/gdt + 1e-9), size(C.X,1)-1) + 1, :);
k_last = -1; rm = NaN; thm = NaN; have = false;
shots = zeros(0,3); fire_prev = 0; last_pulse = -inf;
perr = zeros(n,1); Vabs = zeros(n,1); thlog = zeros(n,1);
useinline = any(o.scale ~= 1);
for k = 1:n
    t = tv(k);
    kk = floor(t/Ts + 1e-9); newsamp = false;
    if kk > k_last
        k_last = kk;
        if ~inbo(kk*Ts)
            Xk = truth(kk*Ts);
            rm = hypot(Xk(1),Xk(2)) + nr(kk+1); thm = atan2(Xk(2),Xk(1)) + nth(kk+1);
            newsamp = true; have = true;
        end
    end
    switch o.meas
        case 'pulse', mv = double(newsamp); r_in = rm; th_in = thm;
        case 'hold',  mv = double(have && ~inbo(t)); r_in = rm; th_in = thm;
        case 'nan',   mv = double(newsamp); if newsamp, r_in = rm; th_in = thm; else, r_in = NaN; th_in = NaN; end
    end
    if ~have, r_in = NaN; th_in = NaN; mv = 0; end
    [V, fc] = ctrl(t, r_in, th_in, mv, x(1), x(2));
    X = truth(t);
    if fc == 1 && fire_prev == 0 && t - last_pulse >= P.t_recharge - 1e-9
        last_pulse = t; dm = ray_miss(x(1), X(1:2));
        shots(end+1,:) = [t dm dm <= P.R_eng]; %#ok<AGROW>
    end
    fire_prev = fc;
    perr(k) = ray_miss(x(1), X(1:2)); Vabs(k) = abs(V); thlog(k) = x(1);
    if k < n
        if useinline, f = @(xx) plant_mm(V, xx, P, o.scale);
        else, f = @(xx) pv(t, V, xx); end
        h = o.dt;
        if strcmp(o.integ,'euler'), x = x + h*f(x);
        else
            k1 = f(x); k2 = f(x+h/2*k1); k3 = f(x+h/2*k2); k4 = f(x+h*k3);
            x = x + h/6*(k1+2*k2+2*k3+k4);
        end
    end
    if any(~isfinite(x)) || abs(x(1)) > 10, break; end
end
GUARDIAN_OVERRIDE = [];
R.id = C.id; R.fam = C.fam; R.shots = shots; R.n_shots = size(shots,1);
R.n_hits = sum(shots(:,3)); R.success = R.n_hits > 0;
R.t1 = NaN; if R.success, R.t1 = shots(find(shots(:,3),1),1); end
R.hit_miss = mean(shots(shots(:,3)==1,2)); R.max_shot_miss = max([shots(:,2); 0]);
R.first_shot_t = NaN; if R.n_shots, R.first_shot_t = shots(1,1); end
m = tv >= 3 & (1:n).' <= k;
R.rms_perp_t3 = sqrt(mean(perr(m).^2));
R.Vmax_after05 = max(Vabs(tv > 0.5 & (1:n).' <= k));
R.trace = [tv(1:k) thlog(1:k) perr(1:k) Vabs(1:k)]; R.xend = x;
R.unstable = any(~isfinite(x)) || abs(x(1)) > 10 || R.rms_perp_t3 > 200;
end

function dx = pv(t, V, x)
[a,b,c] = turret_plant(t, V, x(1), x(2), x(3)); dx = [a;b;c];
end
function dx = plant_mm(V, x, P, s)
Jeq = s(1)*P.J_eq; L = s(2)*P.Lm; b = s(3)*P.b; Kt = s(4)*P.Kt; Ke = s(4)*P.Ke;
dx = [x(2); (P.N*Kt*x(3) - b*x(2))/Jeq; (V - P.Rm*x(3) - Ke*P.N*x(2))/L];
end
function dm = ray_miss(th, p)
u = [cos(th); sin(th)]; s = u.'*p(:);
if s < 0, dm = norm(p); else, dm = abs(-sin(th)*p(1) + cos(th)*p(2)); end
end
