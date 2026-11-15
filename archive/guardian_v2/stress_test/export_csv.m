tags = {'base_track','base_ambush','v_dt10_track','v_dt20_track','v_dt50_track','v_hold_track','v_euler_track','v_th0_track','v_mmhi_track','v_mmlo_track', ...
  'v_dt10_ambush','v_dt20_ambush','v_dt50_ambush','v_hold_ambush','v_euler_ambush','v_th0_ambush','v_mmhi_ambush','v_mmlo_ambush', ...
  'f_wc10_track','f_zi5_track','f_wc10zi5_track','f_hybrid','f_c2hold_track', ...
  'n_full','n_dt50','n_dt50_wf15','n_dt20','n_dt10','n_hold','n_euler','n_th0','n_mmhi','n_mmlo','n_wf15','s_full','s_dt20','s_dt10','s_hold','s_mmhi'};
fid = fopen('stress/summary.csv','w');
fprintf(fid,'tag,n,success_pct,t1_mean,t1_median,t1_p90,t1_max,precision_pct,shots_per_case,hit_miss_m,rms_perp_t3_median,unstable\n');
for k = 1:numel(tags)
  if ~exist(['stress/' tags{k} '_a.mat'],'file'), continue; end
  T = summarize(tags{k},1);
  fprintf(fid,'%s,%d,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.2f,%.3f,%.3f,%d\n', tags{k}, T.n, 100*T.success, T.t1_mean, T.t1_med, T.t1_p90, T.t1_max, 100*T.precision, T.shots, T.hit_miss, T.rms3_med, T.unstable);
end
fclose(fid); type stress/summary.csv
