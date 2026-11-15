function T = summarize(tag, quiet)
%SUMMARIZE  Aggregate metrics for a stress batch tag (stress/<tag>_a/_b.mat)
if nargin < 2, quiet = false; end
RR = {};
for s = {'a','b'}
    f = sprintf('stress/%s_%s.mat', tag, s{1});
    if exist(f,'file'), L = load(f); RR = [RR L.RR]; end %#ok<AGROW>
end
n = numel(RR); succ = zeros(n,1); t1 = nan(n,1); ns = zeros(n,1); nh = zeros(n,1);
hm = nan(n,1); uns = zeros(n,1); fam = cell(n,1); ids = zeros(n,1); rms3 = nan(n,1); fs = nan(n,1);
for k = 1:n
    R = RR{k}; ids(k) = R.id; fam{k} = R.fam; succ(k) = R.success; uns(k) = R.unstable;
    if isfield(R,'t1'), t1(k) = R.t1; ns(k) = R.n_shots; nh(k) = R.n_hits; hm(k) = R.hit_miss; rms3(k) = R.rms_perp_t3; fs(k) = R.first_shot_t; end
end
T.tag = tag; T.n = n; T.success = mean(succ); T.t1_mean = mean(t1(succ==1)); T.t1_med = NaN; if any(succ), T.t1_med = median(t1(succ==1)); end
T.t1_p90 = prctile_(t1(succ==1), 90); T.t1_max = max(t1(succ==1));
T.precision = sum(nh)/max(sum(ns),1); T.shots = mean(ns); T.hit_miss = mean(hm(~isnan(hm)));
T.unstable = sum(uns); T.fail_ids = ids(succ==0).'; T.rms3_med = median(rms3);
T.ids = ids; T.succ = succ; T.t1 = t1; T.fam = fam; T.nh = nh; T.ns = ns; T.hm = hm;
if ~quiet
    printf('%-18s n=%3d succ=%5.1f%%  t1 mean/med/p90/max = %.2f/%.2f/%.2f/%.2f  prec=%5.1f%%  shots=%.1f  hitmiss=%.2fm  rms_perp(t>3) med=%.2fm  unstable=%d\n', ...
        tag, n, 100*T.success, T.t1_mean, T.t1_med, T.t1_p90, T.t1_max, 100*T.precision, T.shots, T.hit_miss, T.rms3_med, T.unstable);
    if any(succ==0), printf('   failed ids: %s\n', mat2str(T.fail_ids)); end
end
end
function p = prctile_(x, q)
x = sort(x(:)); if isempty(x), p = NaN; return; end
p = x(max(1, ceil(q/100*numel(x))));
end
