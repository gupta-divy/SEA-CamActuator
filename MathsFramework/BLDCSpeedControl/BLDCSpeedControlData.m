%% Parameters for BLDC Speed Control Example

% This example shows how to control the rotor speed in a BLDC based 
% electrical drive. An ideal torque source provides the load. The Control 
% subsystem uses a PI-based cascade control structure with an outer speed 
% control loop and an inner dc-link voltage control loop. The dc-link 
% voltage is adjusted through a DC-DC buck converter. The BLDC is fed by 
% a controlled three-phase inverter. The gate signals for the inverter 
% are obtained from hall signals. The simulation uses speed steps. The 
% Scopes subsystem contains scopes that allow you to see the simulation 
% results.

% Copyright 2017-2023 The MathWorks, Inc.

%% Machine Parameters
p    = 14;              % Number of pole pairs
Rs   = 	0.577;            % Stator resistance per phase           [Ohm]
Ls   = 0.000704;           % Stator self-inductance per phase, Ls  [H]
Ms   = 	0.000117;           % Stator mutual inductance, Ms          [H]
psim = 	0.1194;         % Maximum permanent magnet flux linkage [Wb]
Jm   = 	3.2e-6;          % Rotor inertia                         [Kg*m^2]

%% Control Parameters
Ts  = 5e-6;     % Fundamental sample time            [s]
Tsc = 1e-4;     % Sample time for inner control loop [s]
Vdc = 24;       % Maximum DC link voltage            [V]
Kpw = 4.0;     % Proportional gain speed controller
Kiw = 1.0;       % Integrator gain speed controller
Kpv = 0.10282151401042938;      % Proportional gain voltage controller
Kiv = 145.4460906982422;      % Integrator gain voltage controller
