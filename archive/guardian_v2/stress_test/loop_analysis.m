P = guardian_params(); C = design_controller(P);
printf('K=%.0f a=%.3f wn=%.2f zeta=%.3f phi_rest=%.1f\n', C.K, C.lead_a, C.wn, C.zeta_p, C.phi_rest);
w = logspace(-1, 3, 200000); jw = 1j*w;
L = polyval(C.numC,jw)./polyval(C.denC,jw).*polyval(C.numP,jw)./polyval(C.denP,jw);
ph = unwrap(angle(L))*180/pi; mag = abs(L);
ic = find(mag < 1, 1); ip = find(ph < -180, 1);
printf('continuous: wc=%.2f PM=%.1f  GM=%.1f dB at %.1f  Ms=%.2f\n', w(ic), 180+ph(ic), -20*log10(mag(ip)), w(ip), max(abs(1./(1+L))));
% closed-loop poles
cl = conv(C.denC, C.denP) + [zeros(1, numel(conv(C.denC,C.denP))-numel(conv(C.numC,C.numP))) conv(C.numC,C.numP)];
disp('closed-loop poles:'); disp(roots(cl).');
% sampled-data: compensator ZOH-discretised as implemented (state update uses e(k-1)),
% plant ZOH-discretised; check discrete closed-loop stability for several step sizes
for Ts = [0.001 0.005 0.01 0.02 0.03 0.05]
  [Ac,Bc,Cc,Dc] = deal(C.A,C.B,C.Cc,C.D);
  % plant ss  x=[th thd i]
  Ap = [0 1 0; 0 -P.b/P.J_eq P.N*P.Kt/P.J_eq; 0 -P.Ke*P.N/P.Lm -P.Rm/P.Lm]; Bp = [0;0;1/P.Lm];
  M = expm([Ap Bp; zeros(1,4)]*Ts); Apd = M(1:3,1:3); Bpd = M(1:3,4);
  nc = size(Ac,1); Mc = expm([Ac Bc; zeros(1,nc+1)]*Ts); Acd = Mc(1:nc,1:nc); Bcd = Mc(1:nc,nc+1);
  % states: xp(3), xc(nc), eprev(1). e(k) = -th(k) (r=0). u(k)=Cc*xc(k)+Dc*e(k)
  % xc(k+1)= Acd xc(k) + Bcd e(k)   (in code: xc updated with e_prev BEFORE computing u -> xc(k)=Acd xc(k-1)+Bcd e(k-1))
  E = [-1 0 0];
  Acl = [Apd + Bpd*Dc*E, Bpd*Cc; Bcd*E, Acd];
  printf('Ts=%5.3f  max|eig| = %.4f %s\n', Ts, max(abs(eig(Acl))), char('stable'*(max(abs(eig(Acl)))<1) + 0));
end
