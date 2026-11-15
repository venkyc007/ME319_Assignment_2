function C = real_case(id, ka)
%REAL_CASE  stress_case with maneuver acceleration scaled by ka (ka>1 is BEYOND the assigned 20 m/s^2 envelope).
if nargin < 2, ka = 2; end
%STRESS_CASE  Harder, admissible test cases (stand-in for unseen instructor cases).
%   Families (by id):
%     1-40    'weave'  : sinusoidal lateral accel 14-20 m/s^2, 0.5-3 rad/s
%     41-80   'jink'   : bang-bang lateral accel +-(14-20) m/s^2, smoothstep
%                        reversals of 0.15-0.4 s, every 0.4-1.5 s
%     81-110  'close'  : r0 850-1100 m, 120-150 m/s, crossing + weave/jink
%     111-140 'adv'    : jink with reversals timed INSIDE the blackout(s)
%     141-160 'highel' : elevation 45-80 deg, climbing/diving + weave
%   Blackouts: onset >= 1.0 s, each <= 0.5 s; ~1/3 at exactly 1.0 s / 0.5 s;
%   some cases have 2-3 blackouts.
%   Returns C.tg, C.X [x y vx vy ax ay] on a 1 ms grid, C.bo = [on dur; ...].
P = guardian_params();
dt = 1e-3; tg = (0:dt:P.t_end+0.5).'; n = numel(tg);
rand('state', 7919*id); randn('state', 104729*id);
for tries = 1:500
    if id <= 40, fam = 'weave';
    elseif id <= 80, fam = 'jink';
    elseif id <= 110, fam = 'close';
    elseif id <= 140, fam = 'adv';
    else, fam = 'highel';
    end
    % ---- initial geometry
    switch fam
        case 'close',  r0 = 850 + 250*rand; el0 = (5 + 45*rand)*pi/180; spd = 120 + 30*rand;
        case 'highel', r0 = 1000 + 800*rand; el0 = (45 + 35*rand)*pi/180; spd = 80 + 70*rand;
        otherwise,     r0 = 1200 + 750*rand; el0 = (3 + 55*rand)*pi/180; spd = 80 + 70*rand;
    end
    p0 = r0*[cos(el0) sin(el0)];
    los = el0 + pi;                                  % inbound direction
    if strcmp(fam,'close'), hdg = el0 + pi/2*sign(rand-0.5) + 0.4*(rand-0.5);  % crossing
    else, hdg = los + 1.2*(rand-0.5); end
    % ---- blackouts
    r = rand;
    if r < 0.35, bo = [1.0 0.5];
    elseif r < 0.75, bo = [1 + 7*rand, 0.3 + 0.2*rand];
    else
        nb = 2 + (rand > 0.5); bo = zeros(nb,2); t0 = 1.0;
        for k = 1:nb, t0 = t0 + 0.6*rand + (k>1)*0.8; bo(k,:) = [t0, 0.3+0.2*rand]; t0 = t0 + bo(k,2); end
    end
    % ---- lateral-acceleration profile a_lat(t)
    A = ka*(14 + 6*rand); t_on = 0.5 + 1.5*rand;
    if strcmp(fam,'adv'), t_on = 0.3*rand; end
    switch fam
        case {'weave','highel'}
            w = 0.5 + 2.5*rand; ph = 2*pi*rand;
            alat = A*sin(w*tg + ph);
        case {'jink','adv','close'}
            if strcmp(fam,'close') && rand < 0.5
                w = 0.5 + 2.5*rand; alat = A*sin(w*tg + 2*pi*rand);
            else
                sw = []; tt = t_on;
                if strcmp(fam,'adv')     % put a reversal in the middle of every blackout
                    for k = 1:size(bo,1), sw(end+1) = bo(k,1) + bo(k,2)*(0.2+0.5*rand); end
                end
                while tt < P.t_end + 0.5, tt = tt + 0.4 + 1.1*rand; sw(end+1) = tt; end
                sw = sort(sw); Tr = 0.15 + 0.25*rand;
                s = sign(rand-0.5)*ones(n,1); sgn = s(1);
                for k = 1:numel(sw)
                    u = min(max((tg - sw(k))/Tr,0),1); u = 3*u.^2 - 2*u.^3;
                    s = s - 2*sgn*u; sgn = -sgn;
                end
                alat = A*s;
            end
    end
    ramp = min(max((tg - t_on)/0.3,0),1); ramp = 3*ramp.^2 - 2*ramp.^3;
    if strcmp(fam,'adv'), ramp(:) = 1; end
    alat = alat.*ramp;
    % small tangential accel (speed change), keeps |a| <= 20
    atan_ = 0.25*sqrt(max((ka*20)^2 - alat.^2,0)).*sin((0.3+rand)*tg + 2*pi*rand).*(rand<0.6);
    % ---- integrate (1 ms, semi-implicit Euler is plenty at this step)
    X = zeros(n,6); p = p0; v = spd*[cos(hdg) sin(hdg)];
    for k = 1:n
        vh = v/norm(v); nh = [-vh(2) vh(1)];
        a = alat(k)*nh + atan_(k)*vh;
        X(k,:) = [p v a];
        v = v + a*dt; p = p + v*dt;
    end
    m = tg <= P.t_end;
    rr = hypot(X(m,1),X(m,2)); sp = hypot(X(m,3),X(m,4)); aa = hypot(X(m,5),X(m,6));
    el = atan2(X(m,2),X(m,1));
    ok = all(rr >= P.r_min & rr <= P.r_max) && all(sp <= P.v_max) && all(aa <= ka*P.a_max+1e-9) ...
         && all(el > P.theta_min + 2*pi/180 & el < P.theta_max - 2*pi/180);
    if ok, break; end
end
if ~ok, error('real_case %d: no admissible trajectory', id); end
C.id = id; C.fam = fam; C.tg = tg; C.X = X; C.bo = bo;
C.amax = max(aa); C.vmax = max(sp); C.rmin = min(rr);
C.jmax = max(hypot(diff(X(m,5)),diff(X(m,6))))/dt;
C.losrate_max = max(abs(diff(el)))/dt;
end
