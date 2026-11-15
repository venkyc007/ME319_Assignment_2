function T = analyse_sweep(tag, fire_ks, quiet)
% Loads sweep/<tag>_{o,e}.mat. If fire_ks given (track mode), replays the fire gate.
if nargin < 3, quiet = false; end
A = load(['sweep/' tag '_o.mat']); B = load(['sweep/' tag '_e.mat']);
res = [A.res B.res]; [~,o] = sort([res.id]); res = res(o);
P = guardian_params();
if nargin < 2 || isempty(fire_ks), fire_ks = NaN; end
T = [];
for fk = fire_ks
    n = numel(res); first = nan(n,1); nsh = zeros(n,1); nh = zeros(n,1); miss = [];
    for j = 1:n
        if isnan(fk), sh = res(j).shots(:,1:3);
        else
            L = struct('t',res(j).t,'conv',res(j).conv,'front',res(j).front,'miss_est',double(res(j).miss_est), ...
                       'sig_perp',double(res(j).sig_perp),'true_miss',double(res(j).true_miss));
            sh = replay_fire(L, fk, P);
        end
        nsh(j) = size(sh,1); nh(j) = sum(sh(:,3)); miss = [miss; sh(:,2)];
        if nh(j) > 0, first(j) = sh(find(sh(:,3),1),1); end
    end
    r.tag = tag; r.fire_k = fk; r.success = mean(nh > 0); r.t_first_mean = mean(first(~isnan(first)));
    r.t_first_med = median(first(~isnan(first))); r.t_first_p90 = prctile_(first(~isnan(first)), 90);
    r.precision = sum(nh)/max(sum(nsh),1); r.shots = mean(nsh); r.hits = mean(nh);
    r.miss_med = median(miss); r.rms_perp = mean([res.rms_perp]); r.est_rms = mean([res.est_rms]);
    r.failed_ids = [res(nh==0).id]; r.first = first; r.nh = nh; r.nsh = nsh;
    T = [T r]; %#ok<AGROW>
    if ~quiet
        fprintf('%-8s k=%4.2f  success=%3.0f%%  t1 mean=%.2f med=%.2f p90=%.2f  precision=%5.1f%%  shots=%.1f hits=%.1f  med miss=%.2fm  rms_perp=%.2fm est=%.2fm  fails=%s\n', ...
          tag, fk, 100*r.success, r.t_first_mean, r.t_first_med, r.t_first_p90, 100*r.precision, r.shots, r.hits, r.miss_med, r.rms_perp, r.est_rms, mat2str(r.failed_ids));
    end
end
end
function v = prctile_(x, p)
x = sort(x(:)); if isempty(x), v = NaN; return; end
v = x(max(1, ceil(p/100*numel(x))));
end
