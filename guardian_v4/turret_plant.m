function [theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
%TURRET_PLANT  First-principles elevation-drive dynamics (reference plant Gp(s)).
%
%   Mechanical (turret side, motor inertia reflected through N:1 gearbox):
%       J_eq * theta_ddot = N*Kt*i - b*theta_dot - tau_stop
%       J_eq = J_turret + N^2 * Jm
%   Electrical (armature):
%       Lm * di/dt = V_m - Rm*i - Ke*N*theta_dot
%
%   Mechanical end-stops at -10 deg / +85 deg are modelled as a stiff
%   spring-damper that only acts beyond the limits (inactive in normal use).
%   Gravity unbalance is neglected (trunnion assumed balanced) -- state in paper.

persistent P
if isempty(P), P = guardian_params(); end

tau_stop = 0;
k_stop = 1e7; c_stop = 1e5;          % N m/rad, N m s/rad
if theta_turret > P.theta_max
    tau_stop = k_stop*(theta_turret - P.theta_max) + c_stop*max(theta_dot_turret,0);
elseif theta_turret < P.theta_min
    tau_stop = k_stop*(theta_turret - P.theta_min) + c_stop*min(theta_dot_turret,0);
end

theta_ddot_turret = (P.N*P.Kt*i - P.b*theta_dot_turret - tau_stop) / P.J_eq;
i_dot = (V_m - P.Rm*i - P.Ke*P.N*theta_dot_turret) / P.Lm;
end
