% ME 327 Assignment 3, Problem 2 

%Clear the workspace.
clear all; close all; clc;

%%
% Define the angle ranges and buzzer amplitude limits
forward_angle_min = 1;
forward_angle_max = 8;
backward_angle_min = -12;
backward_angle_max = -3;
MIN_BUZZER_AMP = 70;
MAX_BUZZER_AMP = 170;

% Create a range of angle differences for forward and backward and good
% posture range
forward_diff = linspace(forward_angle_min, forward_angle_max, 100);
backward_diff = linspace(backward_angle_min, backward_angle_max, 100);
middle_diff = linspace(backward_angle_max, forward_angle_min, 100);

% Map the forward differences to buzzer amplitudes
forward_buzzStrength = map(forward_diff, forward_angle_min, forward_angle_max, MIN_BUZZER_AMP, MAX_BUZZER_AMP);

% Map the backward differences to buzzer amplitudes
backward_buzzStrength = map(backward_diff, backward_angle_max, backward_angle_min, MIN_BUZZER_AMP, MAX_BUZZER_AMP);

% Good posture buzzer amplitudes
middle_buzzStrength = zeros(size(middle_diff));

% Concatenate
angles = [backward_diff, middle_diff, forward_diff];
response = [backward_buzzStrength, middle_buzzStrength, forward_buzzStrength];

% Plot the results
figure(1);
hold on;
xline(backward_angle_max, 'b--', 'LineWidth', 2)
xline(forward_angle_min, 'r--', 'LineWidth', 2)
plot(backward_diff, backward_buzzStrength * 100/255, 'b-', 'LineWidth', 2)
plot(forward_diff, forward_buzzStrength * 100/255, 'r-', 'LineWidth', 2)
plot(middle_diff, middle_buzzStrength, 'k', 'LineWidth', 2)
xlabel('Measured Upper Back Angle relative to Lower Back (degrees)');
ylabel('PWM Signal Duty Cycle (%)');
title('ERM Amplitude Response Profile');
legend('Backward Bending Threshold', 'Forward Bending Threshold', 'Signal to Back ERM',...
    'Signal to Front ERM', 'Good Posture Region','location', 'southwest');
grid on;
hold off;
printpdf(gcf, 'ERM_Response.pdf')

% Function to simulate Arduino map function
function out = map(x, in_min, in_max, out_min, out_max)
    out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
end

% Function to save plot to pdf
function printpdf(h,outfilename)

set(h, 'PaperUnits','centimeters');
set(h, 'Units','centimeters');
pos=get(h,'Position');
set(h, 'PaperSize', [pos(3) pos(4)]);
set(h, 'PaperPositionMode', 'manual');
set(h, 'PaperPosition',[0 0 pos(3) pos(4)]);
print('-dpdf',outfilename);
end