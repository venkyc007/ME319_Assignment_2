%V4_CHECK  First check of the v4 port. VERIFIED 15 Nov 2026 in GNU Octave 11.3.0:
%   the output below was reproduced exactly (to the printed precision).
%   Expected output (Python prototype, 1 ms RK4, controller every step):
% A   shots=10 hits=10 first_hit=0.302 s mean_miss=0.30 m |V|max=1829 V |i|max=90.6 A
% B   shots=10 hits=10 first_hit=0.302 s mean_miss=0.31 m |V|max=1823 V |i|max=90.6 A
% C   shots=10 hits=10 first_hit=0.309 s mean_miss=0.48 m |V|max=1840 V |i|max=90.9 A
% R1  shots=10 hits=10 first_hit=0.161 s mean_miss=0.19 m |V|max=1669 V |i|max=91.9 A
% R7  shots=10 hits=10 first_hit=0.193 s mean_miss=0.19 m |V|max=1691 V |i|max=90.6 A
%   v3 for comparison: A/B/C 8 shots, 8 hits, first hit 0.44 s, mean miss 0.6-0.8 m.
%   The Python port reproduced v3 exactly, so small differences (a few ms, a few cm)
%   are acceptable; a different shot count or a miss means a porting bug.
for sc = {'A','B','C','R1','R7'}
    clear turret_controller turret_plant
    run_scenario(sc{1});
end
