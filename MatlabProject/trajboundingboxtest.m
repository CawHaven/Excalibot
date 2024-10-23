% Script to calculate trajectory and find bounding box for Excalibot

% Create an instance of Excalibot robot
baseTr = eye(4); % Identity matrix for base transformation
robot = Excalibot(baseTr);

% Define desired position [x, y, z, roll, pitch, yaw]
desiredPosition = [1.0, 0.5, 0.75, pi/4, pi/6, pi/3]; % Example position

% Calculate inverse kinematics to get the required joint angles
targetTransformation = transl(desiredPosition(1), desiredPosition(2), desiredPosition(3)) * ...
                       rpy2tr(desiredPosition(4), desiredPosition(5), desiredPosition(6));

targetJointAngles = robot.model.ikcon(targetTransformation);

% Get the current joint positions
currentJointAngles = robot.model.getpos();

% Calculate the trajectory between the current joint angles and target joint angles
numSteps = 50; % Number of steps for the trajectory
trajectory = jtraj(currentJointAngles, targetJointAngles, numSteps);

% Initialize arrays to store joint positions throughout the trajectory
jointPositions = zeros(numSteps, 6, 3); % [step, joint, x/y/z]

% Loop over the trajectory to find the positions of all joints
for i = 1:numSteps
    % Get the transformation matrix for each joint at step i
    jointTransforms = robot.model.fkine(trajectory(i, :)); % Forward kinematics
    
    % Extract the position of each joint
    for j = 1:6
        jointPos = jointTransforms(j).t; % Extract translation part from SE(3) transformation
        jointPositions(i, j, :) = jointPos; % Store x, y, z
    end
end

% Calculate minimum and maximum values for x, y, and z across all joints and steps
minX = min(jointPositions(:, :, 1), [], 'all');
maxX = max(jointPositions(:, :, 1), [], 'all');
minY = min(jointPositions(:, :, 2), [], 'all');
maxY = max(jointPositions(:, :, 2), [], 'all');
minZ = min(jointPositions(:, :, 3), [], 'all');
maxZ = max(jointPositions(:, :, 3), [], 'all');

% Display the bounding box coordinates
fprintf('Bounding Box Coordinates:\n');
fprintf('Min X: %.2f, Max X: %.2f\n', minX, maxX);
fprintf('Min Y: %.2f, Max Y: %.2f\n', minY, maxY);
fprintf('Min Z: %.2f, Max Z: %.2f\n', minZ, maxZ);

% Optionally, plot the robot trajectory to visualize the motion
for i = 1:numSteps
    robot.model.animate(trajectory(i, :)); % Animate the robot movement
    drawnow();
end
