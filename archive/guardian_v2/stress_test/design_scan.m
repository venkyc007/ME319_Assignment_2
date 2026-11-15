P0 = guardian_params();
Ap = [0 1 0; 0 -P0.b/P0.J_eq P0.N*P0.Kt/P0.J_eq; 0 -P0.Ke*P0.N/P0.Lm -P0.Rm/P0.Lm]; Bp = [0;0;1/P0.Lm];
printf('  wc  zi   lead_a     K     PM    GM(dB)  Ms   slowpole | max|eig| at Ts=10/20/50ms | worst Re over J,L,Kt +-20%%\n');
for wc = [10 15 20]
 for zi = [2 3 5 8]
  P = P0; P.wc = wc; P.z_i = zi; C = design_controller(P);
  w = logspace(-1,3,100000); jw=1j*w;
  L = polyval(C.numC,jw)./polyval(C.denC,jw).*polyval(C.numP,jw)./polyval(C.denP,jw);
  ph = unwrap(angle(L))*180/pi; mg = abs(L); ic = find(mg<1,1); ip = find(ph<-180,1);
  PM = 180+ph(ic); if isempty(ip), GM=Inf; else, GM = -20*log10(mg(ip)); end
  a = conv(C.denC,C.denP); b = conv(C.numC,C.numP); b=[zeros(1,numel(a)-numel(b)) b]; r = roots(a+b);
  sp = max(real(r(abs(imag(r))<1e-6)));
  ee = [];
  for Ts = [0.01 0.02 0.05]
    M = expm([Ap Bp; zeros(1,4)]*Ts); Apd = M(1:3,1:3); Bpd = M(1:3,4);
    nc = size(C.A,1); Mc = expm([C.A C.B; zeros(1,nc+1)]*Ts); Acd = Mc(1:nc,1:nc); Bcd = Mc(1:nc,nc+1);
    E = [-1 0 0]; ee(end+1) = max(abs(eig([Apd + Bpd*C.D*E, Bpd*C.Cc; Bcd*E, Acd])));
  end
  worst = -Inf;
  for sJ=[0.8 1 1.2], for sL=[0.8 1 1.2], for sK=[0.8 1 1.2]
    Q = P; Q.J_eq = sJ*P.J_eq; Q.Lm = sL*P.Lm; Q.Kt = sK*P.Kt; Q.Ke = sK*P.Ke;
    dP = conv([1 0], conv([Q.J_eq Q.b],[Q.Lm Q.Rm]) + [0 0 Q.N^2*Q.Kt*Q.Ke]);
    a = conv(C.denC,dP); b = conv(C.numC,Q.N*Q.Kt); b=[zeros(1,numel(a)-numel(b)) b];
    worst = max(worst, max(real(roots(a+b))));
  end, end, end
  printf('%4d %3d  %6.2f %7.0f  %5.1f  %5.1f  %5.2f  %6.2f  |  %.3f / %.3f / %.3f  |  %6.2f\n', wc, zi, C.lead_a, C.K, PM, GM, max(abs(1./(1+L))), sp, ee, worst);
 end
end
