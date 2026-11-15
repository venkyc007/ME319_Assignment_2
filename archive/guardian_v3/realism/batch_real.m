function batch_real(ids, o, outfile, casefile)
if nargin < 4, casefile = 'stress/cases.mat'; end
load(casefile,'CS');
RR = {};
for id = ids
    try, R = eval_real(CS{id}, o);
    catch err, R = struct('id',id,'fam',CS{id}.fam,'error',err.message,'success',false,'unstable',true); end
    RR{end+1} = R; %#ok<AGROW>
end
save('-v7', outfile, 'RR', 'o');
end
