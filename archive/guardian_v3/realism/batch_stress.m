function batch_stress(ids, o, outfile)
% run eval_stress on cached cases ids with options o; save results array
load('stress/cases.mat','CS');
RR = {};
for id = ids
    try
        R = eval_stress(CS{id}, o);
    catch err
        R = struct('id',id,'fam',CS{id}.fam,'error',err.message,'success',false,'unstable',true);
    end
    RR{end+1} = R; %#ok<AGROW>
    printf('%d %s succ=%d hits=%d/%d t1=%.2f\n', id, CS{id}.fam, R.success, getf(R,'n_hits'), getf(R,'n_shots'), getf(R,'t1')); fflush(stdout);
end
save('-v7', outfile, 'RR', 'o');
end
function v = getf(R,f), if isfield(R,f), v = R.(f); else, v = NaN; end, end
