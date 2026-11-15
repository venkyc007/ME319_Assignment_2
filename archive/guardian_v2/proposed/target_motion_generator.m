function [r_meas, theta_meas, meas_valid, truth] = target_motion_generator(t, scenario)
%TARGET_MOTION_GENERATOR  Target truth + 20 Hz noisy radar (r, theta).
%
%   scenario: 'A' nominal straight-line, 'B' smooth x/y maneuver from t=1 s,
%             'C' = B trajectory + radar loss on [1.0, 1.5) s,
%             'R<n>' (e.g. 'R7') randomised admissible case for Monte-Carlo
%             robustness testing (stand-in for the instructor's eval cases).
%
%   meas_valid = 1 only at a radar sample instant (t = k*0.05) outside a
%   blackout; otherwise the last valid sample is held and meas_valid = 0.
%   4th output (not part of the required interface) returns truth
%   [x y vx vy ax ay] for evaluation/plots.

persistent cache
if isempty(cache), cache = struct('key',{},'S',{}); end
key = char(scenario);
idx = find(strcmp({cache.key}, key), 1);
if isempty(idx)
    S = build_scenario(key);
    cache(end+1) = struct('key',key,'S',S);
    idx = numel(cache);
end
S = cache(idx).S;

% truth by linear interpolation of 1 kHz precomputed grid
X = grid_interp(S, t);
truth = X;

% radar sample logic
Ts = S.Ts;
k  = floor(t/Ts + 1e-9);
on_grid = abs(t - k*Ts) < 1e-6;
in_bo   = @(tt) (tt >= S.bo_on - 1e-9) & (tt < S.bo_on + S.bo_dur - 1e-9);
meas_valid = double(on_grid && ~in_bo(k*Ts));
% last valid sample index (hold)
kk = k;
while kk >= 0 && in_bo(kk*Ts), kk = kk - 1; end
kk = max(kk,0);
Xk = grid_interp(S, kk*Ts);
r_true  = hypot(Xk(1), Xk(2));
th_true = atan2(Xk(2), Xk(1));
r_meas     = r_true  + S.sig_r  * S.nr(kk+1);
theta_meas = th_true + S.sig_th * S.nth(kk+1);
end

% ------------------------------------------------------------------------
function X = grid_interp(S, t)
% linear interpolation on the uniform 1 ms truth grid (fast replacement for interp1)
dt = S.tg(2) - S.tg(1);
u  = min(max(t, 0), S.tg(end)) / dt;
j  = min(floor(u + 1e-9), numel(S.tg) - 2);
w  = u - j;
X  = (1 - w)*S.X(j+1,:) + w*S.X(j+2,:);
end

function S = build_scenario(key)
P = guardian_params();
S.Ts = P.Ts_radar; S.sig_r = P.sig_r; S.sig_th = P.sig_th;
dt = 1e-3; tg = (0:dt:P.t_end+1).';
S.tg = tg;
S.bo_on = inf; S.bo_dur = 0;

switch upper(key(1))
    case {'A','B','C'}
        seed = double(key(1));
        p0 = [1900 450]; v0 = [-120 0];
        if upper(key(1)) == 'A'
            acc = zeros(numel(tg),2);
        else
            w = smooth_on(tg, 1.0, 0.5);         % C^1 onset starting at t = 1 s
            acc = [12*w.*sin(1.2*(tg-1)), 12*w.*sin(1.6*(tg-1))];
        end
        if upper(key(1)) == 'C', S.bo_on = 1.0; S.bo_dur = 0.5; end
        X = integrate(tg, p0, v0, acc);
    case 'R'
        seed = 1000 + str2double(key(2:end));
        st = mod(seed*2654435761, 2^32);
        ok = false; tries = 0;
        while ~ok
            tries = tries + 1;
            [u1, st] = lcg(st, 2);
            r0  = 1400 + 550*u1(1);  el0 = (3 + 50*u1(2))*pi/180;
            p0  = r0*[cos(el0) sin(el0)];
            [u1, st] = lcg(st, 1);
            spd = 60 + 85*u1(1);
            [u1, st] = lcg(st, 1);
            hdg = pi + (-0.6 + 1.2*u1(1));            % roughly inbound
            v0  = spd*[cos(hdg) sin(hdg)];
            [u1, st] = lcg(st, 1);
            t_on = 0.5 + 2.5*u1(1);
            [u1, st] = lcg(st, 1);
            w = smooth_on(tg, t_on, 0.3 + 0.5*u1(1));
            [tf_, st] = lcg(st, 4); [tp_, st] = lcg(st, 4); [ta_, st] = lcg(st, 4);
            f = 0.4 + 2.0*reshape(tf_,2,2); ph = 2*pi*reshape(tp_,2,2); amp = reshape(ta_,2,2);
            ax = amp(1,1)*sin(f(1,1)*tg+ph(1,1)) + amp(1,2)*sin(f(1,2)*tg+ph(1,2));
            ay = amp(2,1)*sin(f(2,1)*tg+ph(2,1)) + amp(2,2)*sin(f(2,2)*tg+ph(2,2));
            acc = [ax ay];
            [u1, st] = lcg(st, 1);
            acc = acc / max(hypot(acc(:,1),acc(:,2))) * (8 + 11.5*u1(1));
            acc = acc .* [w w];
            X = integrate(tg, p0, v0, acc);
            ok = check_env(X, tg, P);
            if tries > 200, error('could not build admissible random case'); end
        end
        [u1, st] = lcg(st, 1);
        S.bo_on  = 1.0 + 6*u1(1);
        [u1, st] = lcg(st, 1);
        S.bo_dur = 0.5*u1(1);
    otherwise
        error('unknown scenario %s', key);
end
if ~check_env(X, tg, P)
    warning('scenario %s violates the operating envelope', key);
end
S.X = X;
nK = ceil((P.t_end+1)/S.Ts) + 2;
st2 = mod((seed+77)*2654435761, 2^32);
[S.nr, st2] = lcg(st2, nK, 'n'); [S.nth, st2] = lcg(st2, nK, 'n');
end

function w = smooth_on(tg, t0, T)
u = min(max((tg - t0)/T, 0), 1);
w = 3*u.^2 - 2*u.^3;                     % smoothstep: continuous accel, bounded jerk
end

function X = integrate(tg, p0, v0, acc)
dt = tg(2)-tg(1);
v = [v0(1) + [0; cumsum((acc(1:end-1,1)+acc(2:end,1))/2*dt)], ...
     v0(2) + [0; cumsum((acc(1:end-1,2)+acc(2:end,2))/2*dt)]];
p = [p0(1) + [0; cumsum((v(1:end-1,1)+v(2:end,1))/2*dt)], ...
     p0(2) + [0; cumsum((v(1:end-1,2)+v(2:end,2))/2*dt)]];
X = [p v acc];
end

function ok = check_env(X, tg, P)
m = tg <= P.t_end;
r = hypot(X(m,1),X(m,2)); sp = hypot(X(m,3),X(m,4)); a = hypot(X(m,5),X(m,6));
el = atan2(X(m,2),X(m,1));
ok = all(r >= P.r_min & r <= P.r_max) && all(sp <= P.v_max) && all(a <= P.a_max) ...
     && all(el > P.theta_min + 2*pi/180 & el < P.theta_max - 2*pi/180) && all(X(m,2) > 0);
end

function [u, st] = lcg(st, n, mode)
% deterministic uniform/normal generator, identical in MATLAB and Octave
if nargin < 3, mode = 'u'; end
u = zeros(n,1);
for kk = 1:n
    st = mod(st*1664525 + 1013904223, 2^32);  u(kk) = (st+0.5)/2^32;
end
if mode == 'n'
    u2 = zeros(n,1);
    for kk = 1:n
        st = mod(st*1664525 + 1013904223, 2^32);  u2(kk) = (st+0.5)/2^32;
    end
    u = sqrt(-2*log(u)).*cos(2*pi*u2);
end
end
