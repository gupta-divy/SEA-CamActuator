k = const.beam_stiffness;
T_ang = ang(1:length(ang)-1);
tau = cam_torque(1:length(cam_torque)-1);
del_theta0 = const.init_deflection;
del_theta = beam_defl;


Sat_force_min = force(1);
Sat_force_max = force(end);
Sat_angle_min = ang(1);
Sat_angle_max = ang(end);



diff_del = diff(del_theta)/dTheta;
acc = tau-k*diff_del.*(T_ang+del_theta0);
% J = 0.0001;

diff_cable_len = diff(cable_len)/dTheta;
dd_cable_len = diff(diff_cable_len)/dTheta;

J = (30*55^2+3919.56)/1000/1e6;

Kt = 0.13; % torque constant
R = 32e-3; % cable wheel radius (m)

Jm = 32*1e-7; % Motor inertia (kg*m^2)
Jw = 12778*1e-9; % Cable wheel inertia (kg*m^2) (not including metal screws)

N = 8;  % Reduction ratio