%% BLDC Speed Control
% 
% This example shows how to control the rotor speed in a BLDC based 
% electrical drive. An ideal torque source provides the load. The Control 
% subsystem uses a PI-based cascade control structure with an outer speed 
% control loop and an inner dc-link voltage control loop. The dc-link 
% voltage is adjusted through a DC-DC buck converter. The BLDC is fed by 
% a controlled three-phase inverter. The gate signals for the inverter 
% are obtained from hall signals. The simulation uses speed steps. The 
% Scopes subsystem contains scopes that allow you to see the simulation 
% results.
% 

% Copyright 2017-2023 The MathWorks, Inc.

%% Model

open_system('BLDCSpeedControl')

set_param(find_system('BLDCSpeedControl','FindAll', 'on','type','annotation','Tag','ModelFeatures'),'Interpreter','off')

%% Simulation Results from Simscape Logging
%%
%
% The plot below shows the requested and measured speed for the
% test and the phase currents in the electric drive.
%


BLDCSpeedControlPlotMotorSpeed;

%%

