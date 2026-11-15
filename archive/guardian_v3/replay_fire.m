function sh = replay_fire(L, fire_k, P)
% Offline replay of the covariance-gated fire logic (valid in 'track' mode,
% where firing does not feed back into the turret trajectory).
% Returns shots [t, true_miss, hit].
sh = zeros(0,3); last = -inf; prev = 0;
for k = 1:numel(L.t)
    t = L.t(k); fc = 0;
    if L.conv(k) && L.front(k) && (t - last) >= P.t_recharge + 1e-6 && ...
       (L.miss_est(k) + fire_k*L.sig_perp(k) <= P.R_eng) && prev == 0
        fc = 1; last = t; sh(end+1,:) = [t, L.true_miss(k), L.true_miss(k) <= P.R_eng];
    end
    prev = fc;
end
end
