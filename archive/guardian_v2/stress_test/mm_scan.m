function mm_scan(wc, zeta_n)
P0 = guardian_params(); if nargin>0, P0.wc = wc; end; if nargin>1, P0.zeta_notch_den = zeta_n; end
C = design_controller(P0);
sc = [0.7 0.8 0.9 1.0 1.1 1.2 1.3];
printf('wc=%g  max Re(closed-loop pole); rows = J scale, cols = L scale\n      ', P0.wc); printf('%7.1f', sc); printf('\n');
for sJ = sc
  printf('J%4.1f ', sJ);
  for sL = sc
    P = P0; P.J_eq = sJ*P0.J_eq; P.Lm = sL*P0.Lm;
    denP = conv([1 0], conv([P.J_eq P.b],[P.Lm P.Rm]) + [0 0 P.N^2*P.Kt*P.Ke]); numP = P.N*P.Kt;
    a = conv(C.denC, denP); b = conv(C.numC, numP); b = [zeros(1,numel(a)-numel(b)) b];
    printf('%7.2f', max(real(roots(a+b))));
  end
  printf('\n');
end
% individual params
for f = {'b','Kt','Rm'}
  for s = [0.8 1.2]
    P = P0; P.(f{1}) = s*P0.(f{1}); if strcmp(f{1},'Kt'), P.Ke = s*P0.Ke; end
    denP = conv([1 0], conv([P.J_eq P.b],[P.Lm P.Rm]) + [0 0 P.N^2*P.Kt*P.Ke]); numP = P.N*P.Kt;
    a = conv(C.denC, denP); bb = conv(C.numC, numP); bb = [zeros(1,numel(a)-numel(bb)) bb];
    printf('%s x%.1f : max Re = %.2f\n', f{1}, s, max(real(roots(a+bb))));
  end
end
end
