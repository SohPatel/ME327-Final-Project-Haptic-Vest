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

% Create a range of angle differences for forward and backward
forward_diff = linspace(forward_angle_min, forward_angle_max, 100);
backward_diff = linspace(backward_angle_min, backward_angle_max, 100);

% Map the forward differences to buzzer amplitudes
forward_buzzStrength = map(forward_diff, forward_angle_min, forward_angle_max, MIN_BUZZER_AMP, MAX_BUZZER_AMP);

% Map the backward differences to buzzer amplitudes
backward_buzzStrength = map(backward_diff, backward_angle_max, backward_angle_min, MIN_BUZZER_AMP, MAX_BUZZER_AMP);

% Plot the results
figure(1);
hold on;
plot(forward_diff, forward_buzzStrength, 'LineWidth', 2);
plot(backward_diff, backward_buzzStrength, 'LineWidth', 2);
xlabel('Relative Difference Change Between Upper and Lower Back Angle (degrees)');
ylabel('PWM Signal Duty Cycle');
title('ERM Amplitude Response Profile');
legend('Forward Bending (Slouching)', 'Backward Bending (Hyperextending)', 'location', 'north');
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