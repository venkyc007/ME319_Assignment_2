P0 = guardian_params();
Ap = [0 1 0; 0 -P0.b/P0.J_eq P0.N*P0.Kt/P0.J_eq; 0 -P0.Ke*P0.N/P0.Lm -P0.Rm/P0.Lm]; Bp = [0;0;1/P0.Lm];
printf('%6s', 'wc\Ts'); Tss=[0.005 0.01 0.02 0.025 0.03 0.04 0.05]; printf('%8.3f', Tss); printf('\n');
for wc = [4 6 8 10 12 15 20]
  P = P0; P.wc = wc; C = design_controller(P); printf('%6d', wc);
  for Ts = Tss
    M = expm([Ap Bp; zeros(1,4)]*Ts); Apd = M(1:3,1:3); Bpd = M(1:3,4);
    nc = size(C.A,1); Mc = expm([C.A C.B; zeros(1,nc+1)]*Ts); Acd = Mc(1:nc,1:nc); Bcd = Mc(1:nc,nc+1);
    E = [-1 0 0]; Acl = [Apd + Bpd*C.D*E, Bpd*C.Cc; Bcd*E, Acd];
    printf('%8.3f', max(abs(eig(Acl))));
  end
  printf('\n');
end
