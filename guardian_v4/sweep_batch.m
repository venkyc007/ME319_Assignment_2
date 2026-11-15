function sweep_batch(mode, ids, ovr, outfile)
% Runs cases R<ids> with overrides; saves compact per-case logs for replay.
res = struct('id',{},'shots',{},'t',{},'conv',{},'front',{},'miss_est',{},'sig_perp',{},'true_miss',{},'rms_perp',{},'maxV',{},'est_rms',{});
for j = 1:numel(ids)
    R = runmode(sprintf('R%d',ids(j)), mode, false, ovr);
    L = R.log; d = 10;        % keep every 10th sample for plots; full for replay fields
    res(j).id = ids(j); res(j).shots = R.shots;
    res(j).t = L.t; res(j).conv = logical(L.conv); res(j).front = logical(L.front);
    res(j).miss_est = single(L.miss_est); res(j).sig_perp = single(L.sig_perp); res(j).true_miss = single(L.true_miss);
    res(j).rms_perp = R.rms_perp_after2s; res(j).maxV = R.max_V; res(j).est_rms = R.rms_pos_est_after1s;
end
save('-binary', outfile, 'res', 'ovr', 'mode');
end
