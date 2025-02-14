% Code to plot simulation results from BLDCSpeedControl
%% Plot Description:
%
% The plot below shows the requested and measured speed for the
% test and the phase currents in the electric drive.

% Copyright 2017-2023 The MathWorks, Inc.

% Generate simulation results if they don't exist
if ~exist('simlog_BLDCSpeedControl', 'var') || ...
        simlogNeedsUpdate(simlog_BLDCSpeedControl, 'BLDCSpeedControl') 
    sim('BLDCSpeedControl')
    % Model StopFcn callback adds a timestamp to the Simscape simulation data log
end


% Reuse figure if it exists, else create new figure
if ~exist('h1_BLDCSpeedControl', 'var') || ...
        ~isgraphics(h1_BLDCSpeedControl, 'figure')
    h1_BLDCSpeedControl = figure('Name', 'BLDCSpeedControl');
end
figure(h1_BLDCSpeedControl)
clf(h1_BLDCSpeedControl)

% Get simulation results
simlog_t = simlog_BLDCSpeedControl.BLDC.R.w.series.time;
simlog_wMot = simlog_BLDCSpeedControl.BLDC.R.w.series.values('rpm');
simlog_wRef = logsout_BLDCSpeedControl.get('speed_request');
if strcmp(get_param([bdroot '/BLDC'],'winding_type'), 'ee.enum.statorconnection.wye') || strcmp(get_param([bdroot '/BLDC'],'winding_type'), '1')
    simlog_iMot = simlog_BLDCSpeedControl.Sensing_iabc.Current_Sensor.I.series.values('A');
elseif strcmp(get_param([bdroot '/BLDC'],'winding_type'), 'ee.enum.statorconnection.delta') || strcmp(get_param([bdroot '/BLDC'],'winding_type'), '2')
    simlog_iMot = simlog_BLDCSpeedControl.Sensing_iabc.Current_Sensor.I.series.values('A')./sqrt(3);
end

% Plot results
simlog_handles(1) = subplot(2, 1, 1);
plot(simlog_t, simlog_wMot, 'LineWidth', 1)
hold on
plot(simlog_wRef.Values.Time, simlog_wRef.Values.Data, 'LineWidth', 1)
hold off
grid on
title('Motor Speed')
ylabel('Speed (RPM)')
legend({'Measured','Reference'},'Location','Best');

simlog_handles(2) = subplot(2, 1, 2);
plot(simlog_t, simlog_iMot(:,1))
hold on
plot(simlog_t, simlog_iMot(:,2))
plot(simlog_t, simlog_iMot(:,3))
hold off
grid on
title('Phase Currents')
ylabel('Current (A)')
xlabel('Time (s)')

linkaxes(simlog_handles, 'x')

% Remove temporary variables
clear simlog_t simlog_handles temp_colororder
clear simlog_wRef simlog_wMot simlog_iMot
