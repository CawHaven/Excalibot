clc;
clear;

robot = Excalibot(); % Initialize the robot
robot.model.fkine(robot.model.getpos);
hold on; % Keep the plot

disp('Press Enter to continue...');
pause;  % Wait for user input

% Define joint angles for movement (in radians)
jointAngles = zeros(1, 6); % Initialize to 0 for all 6 joints

% Define target angles in radians
startAngle = deg2rad(0);
endAngle = deg2rad(-60);

% Move from 0 to -60 degrees for all joints
for angle = linspace(startAngle, endAngle, 100)
    jointAngles(:) = angle; % Set current joint angles for all joints
    robot.model.animate(jointAngles); % Animate the robot
    pause(0.05); % Pause for smooth animation
end

% Move back from -60 to 0 degrees for all joints
for angle = linspace(endAngle, startAngle, 100)
    jointAngles(:) = angle; % Set current joint angles for all joints
    robot.model.animate(jointAngles); % Animate the robot
    pause(0.05); % Pause for smooth animation
end

hold off; % Release the plot hold
