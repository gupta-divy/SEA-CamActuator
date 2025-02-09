k = const.beam_stiffness;
T_ang = ang(1:length(ang)-1);
tau = cam_torque(1:length(cam_torque)-1);
del_theta0 = const.init_deflection;
del_theta = beam_defl;

diff_del = diff(del_theta)/dTheta;
acc = tau-k*diff_del.*(T_ang+del_theta0);
J = 0.0001;