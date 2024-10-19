clc;
clear;

robot = Excalibot(); % Initialize the robot
robot.model.fkine(robot.model.getpos);
hold on; % Keep the plot

disp('Press Enter to continue...');
pause;  % Wait for user input

% Move to the specified position
robot.MoveTo([0.5, 0.5, 0.5, 0, 0, 0]);

hold off; % Release the plot hold
