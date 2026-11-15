function R = run_scenario(scenario, dt, verbose)
%RUN_SCENARIO  Closed-loop simulation of one engagement, 0 <= t <= t_end.
%   R = run_scenario('A')  -- fixed-step RK4 at dt (default 1 ms) for the
%   plant, controller called every step (ZOH on V_m), radar at 20 Hz.
%   Engagement scoring follows the brief: pulse on a 0->1 transition of
%   fire_cmd, >= 1 s apart; hit if the beam ray passes within R_eng of truth.

if nargin < 2 || isempty(dt), dt = 1e-3; end
if nargin < 3, verbose = true; end
P = guardian_params();
clear turret_controller            % reset persistent controller state

tv = (0:dt:P.t_end).'; n = numel(tv);
x = [0; 0; 0];                     % [theta; theta_dot; i] -- turret starts level, at rest
L = struct('t',tv,'th',zeros(n,1),'thd',zeros(n,1),'i',zeros(n,1),'V',zeros(n,1), ...
    'th_ref',nan(n,1),'th_hat',nan(n,1),'th_true',zeros(n,1),'r_true',zeros(n,1), ...
    'X',zeros(n,6),'xhat',nan(n,6),'xpred',nan(n,6),'meas',nan(n,2),'valid',zeros(n,1), ...
    'miss_est',nan(n,1),'sig_perp',nan(n,1),'fire',zeros(n,1), ...
    'conv',zeros(n,1),'front',zeros(n,1),'true_miss',nan(n,1));
shots = zeros(0,4);  % [t, true_miss, hit, est_miss]
fire_prev = 0; last_pulse = -inf;

for k = 1:n
    t = tv(k);
    [rm, thm, mv, X] = target_motion_generator(t, scenario);
    [V, fc, dbg] = turret_controller(t, rm, thm, mv, x(1), x(2));

    % --- engagement scoring (independent of controller's own bookkeeping)
    if fc == 1 && fire_prev == 0
        if t - last_pulse >= P.t_recharge - 1e-9
            last_pulse = t;
            d = ray_miss(x(1), X(1:2));
            shots(end+1,:) = [t, d, d <= P.R_eng, dbg.miss_est]; %#ok<AGROW>
        end
    end
    fire_prev = fc;

    % --- log
    L.th(k) = x(1); L.thd(k) = x(2); L.i(k) = x(3); L.V(k) = V;
    L.th_ref(k) = dbg.th_ref; L.th_hat(k) = dbg.th_hat;
    L.X(k,:) = X; L.th_true(k) = atan2(X(2),X(1)); L.r_true(k) = hypot(X(1),X(2));
    L.xhat(k,:) = dbg.x.'; L.xpred(k,:) = dbg.xp.';
    if mv, L.meas(k,:) = [rm thm]; end
    L.valid(k) = mv; L.miss_est(k) = dbg.miss_est; L.sig_perp(k) = dbg.sig_perp; L.fire(k) = fc;
    L.conv(k) = dbg.converged; L.front(k) = dbg.in_front; L.true_miss(k) = ray_miss(x(1), X(1:2));

    % --- plant, RK4 with V held over the step
    if k < n
        f = @(tt,xx) plant_vec(tt, V, xx);
        k1 = f(t,x); k2 = f(t+dt/2, x+dt/2*k1); k3 = f(t+dt/2, x+dt/2*k2); k4 = f(t+dt, x+dt*k3);
        x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    end
end

R.scenario = char(scenario); R.log = L; R.shots = shots;
R.n_shots = size(shots,1); R.n_hits = sum(shots(:,3));
R.t_intercept = NaN;
if R.n_hits > 0, R.t_intercept = shots(find(shots(:,3),1),1); end
R.mean_miss = mean(shots(:,2));
% pointing error w.r.t. the *current* true target (what matters for a hit)
R.point_err = (L.th - L.th_true);
R.perp_miss = abs(sin(R.point_err)).*L.r_true;
m = L.t >= 2;
R.rms_perp_after2s = sqrt(mean(R.perp_miss(m).^2));
est_err = hypot(L.xhat(:,1)-L.X(:,1), L.xhat(:,2)-L.X(:,2));
R.rms_pos_est_after1s = sqrt(mean(est_err(L.t>=1 & ~isnan(est_err)).^2));
R.max_V = max(abs(L.V)); R.max_i = max(abs(L.i));

if verbose
    fprintf('[%s] shots=%d hits=%d first_hit=%.2fs mean_miss=%.2fm rms_perp(t>2)=%.2fm est_rms=%.2fm |V|max=%.0fV |i|max=%.1fA\n', ...
        R.scenario, R.n_shots, R.n_hits, R.t_intercept, R.mean_miss, R.rms_perp_after2s, ...
        R.rms_pos_est_after1s, R.max_V, R.max_i);
end
end

function dx = plant_vec(t, V, x)
[a, b, c] = turret_plant(t, V, x(1), x(2), x(3));
dx = [a; b; c];
end

function d = ray_miss(th, p)
u = [cos(th); sin(th)];
s = u.'*p(:);
if s < 0, d = norm(p); else, d = abs(-sin(th)*p(1) + cos(th)*p(2)); end
end
