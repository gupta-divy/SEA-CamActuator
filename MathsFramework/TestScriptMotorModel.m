% Create a new Simulink model
modelName = 'PMSM_FOC_Model';
new_system(modelName);
open_system(modelName);

% Add PMSM block
pmsm = add_block('simscape/Machines/Permanent Magnet Synchronous Machine', ...
    [modelName, '/PMSM'], 'Position', [150, 100, 400, 300]);

% Set PMSM parameters
set_param(pmsm, 'Number_of_pole_pairs', '14');
set_param(pmsm, 'Stator_resistance', '0.000577');
set_param(pmsm, 'Stator_inductance', '0.000704');
set_param(pmsm, 'Back_EMF_constant', '12.5');
set_param(pmsm, 'Torque_constant', '0.13');
set_param(pmsm, 'Inertia', '3.2e-6');
set_param(pmsm, 'Initial_rotor_speed', '0');

% Add Three-Phase Inverter
inverter = add_block('simscape/Power Electronics/Three-Phase Inverter', ...
    [modelName, '/Three-Phase Inverter'], 'Position', [450, 100, 700, 300]);

% Add FOC Control Subsystem
foc_control = add_block('simulink/Subsystem', ...
    [modelName, '/FOC Control'], 'Position', [750, 100, 1000, 300]);

% Add Load Torque block
load_torque = add_block('simscape/Foundation/Torque Source', ...
    [modelName, '/Load Torque'], 'Position', [150, 350, 300, 450]);
set_param(load_torque, 'Torque', '0.625'); % Adjusted for gear ratio

% Add Gearbox block
gearbox = add_block('simscape/Driveline/Simple Gear', ...
    [modelName, '/Gearbox'], 'Position', [350, 350, 500, 450]);
set_param(gearbox, 'GearRatio', '8');

% Connect Blocks
add_line(modelName, 'Three-Phase Inverter/1', 'PMSM/1');
add_line(modelName, 'Three-Phase Inverter/2', 'PMSM/2');
add_line(modelName, 'Three-Phase Inverter/3', 'PMSM/3');
add_line(modelName, 'PMSM/4', 'Gearbox/1');
add_line(modelName, 'Gearbox/2', 'Load Torque/1');

% Save and Simulate
save_system(modelName);
sim(modelName);
