function T = summarize_real(tag, quiet)
%SUMMARIZE_REAL  summarize + stricter kill radii (0.5 m, 1 m) from logged shot misses.
if nargin < 2, quiet = false; end
T = summarize(tag, true);
RR = {};
for s = {'a','b'}
    f = sprintf('stress/%s_%s.mat', tag, s{1});
    if exist(f,'file'), L = load(f); RR = [RR L.RR]; end %#ok<AGROW>
end
n = numel(RR); s05 = zeros(n,1); s10 = zeros(n,1); allm = [];
for k = 1:n
    if isfield(RR{k},'shots') && ~isempty(RR{k}.shots)
        m = RR{k}.shots(:,2); allm = [allm; m]; %#ok<AGROW>
        s05(k) = any(m <= 0.5); s10(k) = any(m <= 1.0);
    end
end
T.succ05 = mean(s05); T.succ10 = mean(s10);
T.shot05 = mean(allm <= 0.5); T.shot10 = mean(allm <= 1.0); T.miss_med = median(allm);
if ~quiet
    printf('%-22s n=%3d hit(2m)=%5.1f%% t1=%5.2f prec=%5.1f%% unstable=%2d | med miss=%5.2fm | shots<=1m %5.1f%% <=0.5m %5.1f%% | cases w/ shot<=0.5m %5.1f%%\n', ...
      tag, T.n, 100*T.success, T.t1_mean, 100*T.precision, T.unstable, T.miss_med, 100*T.shot10, 100*T.shot05, 100*T.succ05);
end
end
