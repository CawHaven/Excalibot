%% 1. Initialize Environment
clc; clear; close all;
figure;
hold on; 
axis equal;
grid on;
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-2.8, 2.7]); % Set X-axis limits meters
ylim([-2.25, 2.6]); % Set Y-axis limits meter
zlim([0, 2.5]);  % Set Z-axis limits meters
axis manual;

% Add lighting to the scene
camlight('headlight'); % Add a headlight for better visualization
lighting gouraud;

% Initialize log file
transformLogFile = fopen('transform_log.txt', 'w');
shading faceted;

%% Import Static Assets

%%floor model
floorfile = 'Ground.ply';
floorPos = [0,0,0];
PlaceObject(floorfile, floorPos);
% 
fencefile = 'Fencing.ply';
fencePos = [0,0,0];
PlaceObject(fencefile, fencePos);

%% Load Sword PLY data
[swordFaces, swordVertices, swordData] = plyread('Sword.ply', 'tri');  % Load the sword mesh

% Define the desired sword spawn position (1.1275, 0.51221, 0.63283)
swordSpawnPosition = transl(1.1275, 0.51221, 0.63283);  % Translation matrix for sword position

% Apply the translation to the sword vertices
swordVerticesTransformed = [swordVertices, ones(size(swordVertices, 1), 1)] * swordSpawnPosition';  % Apply the translation

% Initially create the patch using the transformed sword vertices
swordPatch = patch('Faces', swordFaces, 'Vertices', swordVerticesTransformed(:, 1:3), ...
    'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'none');  % Default gray color

%%
CobotPedestalfile = 'URRobotPedestal.ply';
CobotPedestalPos = [0,0,0];
PlaceObject(CobotPedestalfile, CobotPedestalPos);

waterfile = 'WaterBucket.ply';
waterPos = [0.15,0,0];
PlaceObject(waterfile, waterPos);

furnacefile = 'Furnace.ply';
furnacePos = [0,0,0];
PlaceObject(furnacefile, furnacePos);
%%
base = transl(-1.3134, -0.61995, 0.58111)
URrobot = UR5(base);
URrobot.model.fkine(URrobot.model.getpos);
initialPose = [pi/2, -pi/2, 0, 0, 0, 0];
URrobot.model.animate(initialPose);
drawnow;


baseTr = transl(-0.06748, 0.3819, 0) * trotz(pi);
robot = Excalibot(baseTr); % Initialize the robot
robot.model.fkine(robot.model.getpos);
hold on; % Keep the plot

disp('Press Enter to move to the sword...');
pause;  % Wait for user input
MoveRobotToSword(robot, swordPatch, swordVertices);  % Call the function to move to the sword and put in furnace

disp('Press Enter to move to the water...');
pause;  % Wait for user input
MoveToWater(robot, swordPatch, swordVertices);  % Call the function to move to the water bucket for quenching

disp('Press Enter to move to the inspection station...');
pause;  % Wait for user input
MoveToInspection(robot, swordPatch, swordVertices);  % Call the function to move to the inspection station with COBOT

disp('Press Enter to start scan..');
MoveCOBOTtoInspection(URrobot);

pause
disp('Press Enter to move to dropoff final position..');
MoveToDropOff(robot, swordPatch, swordVertices);


%%
function MoveRobotToSword(robot, swordPatch, swordVertices)
    % Define waypoints for the robot
    first_point = [1.121, 0.52165, 0.82457, 0, 0, 0];  % Above the sword
    second_point = [1.121, 0.52165, 0.7, 0, 0, 0];  % Down to the sword
    third_point = first_point;  % Back up to the first point
    fourth_point = [0.836, -0.16782, 0.9, 0, 0, 0];  % Move to another location
    fifth_point = [0.836, -0.65, 0.9, 0, 0, 0];  % Final location
    trackSword = false;  % Start with sword tracking off

    % Move to the first point (above the sword)
    disp('Moving to the first point (above the sword)...');
    robot.MoveTo(first_point, trackSword, swordPatch, swordVertices);


    % Move down to the second point (on the sword)
    disp('Moving down to the second point (on the sword)...');
    robot.MoveTo(second_point, trackSword, swordPatch, swordVertices);
    pause(0.5);  % Wait for 1 second to simulate gripping the sword

    % Enable sword tracking and move back up
    disp('Gripping the sword and moving back up...');
    robot.EnableGripping();  % Enable sword tracking (gripping)
    robot.MoveTo(third_point, true, swordPatch, swordVertices);  % Start tracking the sword


    % Move to the third point
    disp('Moving to the third point...');
    robot.MoveTo(fourth_point, true, swordPatch, swordVertices);  % Continue tracking


    % Move to the final point
    disp('Moving to the final point...');
    robot.MoveTo(fifth_point, true, swordPatch, swordVertices);  % Continue tracking
    disp('Sword Heating Up');
end

function MoveToWater(robot, swordPatch, swordVertices)
    % Initialize trackSword variable (always true for tracking)
    trackSword = true;  % Enable sword tracking for all points

    % Define waypoints for the robot
    first_point = [0.836, 0.1, 0.9, 0, 0, 0];  % First point (above the water bucket)
    second_point =  [0.83, 0.1, 1.5, 0, 0, 0];
    third_point = [0, -0.8, 1.5, 0, 0, 0];  % Down to the water bucket
    fourth_point = [0, -0.8, 0.7, 0, 0, 0];   % Final position in the water bucket
  
    % Move to the first point (above the water bucket)
    disp('Moving to the first point (above the water bucket)...');
    robot.MoveTo(first_point, trackSword, swordPatch, swordVertices);
    % Wait for half a second

    % Move down to the second point (near the water bucket)
    disp('Moving down to the second point (near the water bucket)...');
    robot.MoveTo(second_point, trackSword, swordPatch, swordVertices);

    % Move to the third point (final position in the water bucket)
    disp('Moving to the third point (in the water bucket)...');
    robot.MoveTo(third_point, trackSword, swordPatch, swordVertices);
    pause(0.5);  % Wait for half a second

    %    % Move to the third point (final position in the water bucket)
    disp('Moving to the third point (in the water bucket)...');
    robot.MoveTo(fourth_point, trackSword, swordPatch, swordVertices); % Wait for half a second

end

function MoveToInspection(robot, swordPatch, swordVertices)
    % Initialize trackSword variable (always true for tracking)
    trackSword = true;  % Enable sword tracking for all points

    % Define waypoints for the robot
    first_point = [0, -0.8, 1.5, 0, 0, 0];
    second_point =  [-0.75, -0.459, 0.831, 0, 0, 0];

  
    % Move to the first point (above the water bucket)
    disp('Moving to the first point (above the water bucket)...');
    robot.MoveTo(first_point, trackSword, swordPatch, swordVertices);
    % Wait for half a second

    % Move down to the second point (near the water bucket)
    disp('Moving down to the second point (near the water bucket)...');
    robot.MoveTo(second_point, trackSword, swordPatch, swordVertices);

end

function MoveCOBOTtoInspection(URrobot)
    % Define the waypoints for the robot in 3D space
    second_point = [-0.75, -0.6, 0.941];  % First point in space (intermediate)
    third_point = [-0.75, -1.2, 0.941];   % Second point in space (final)
    
    % Define the downward orientation in roll, pitch, yaw (pointing down)
    desiredOrientation = rpy2tr(0, pi, 0);  % Roll=0, Pitch=-pi/2, Yaw=0 (points end effector downwards)
    
    % Create transformation matrices for the first and second points with downward orientation
    firstTransform = transl(second_point) * desiredOrientation;  % First point with orientation
    secondTransform = transl(third_point) * desiredOrientation;  % Second point with orientation
    
    % Save the current joint configuration of the robot (starting position)
    originalConfig = URrobot.model.getpos();  
    
    % Compute the joint configuration for the first and second points using inverse kinematics
    firstConfig = URrobot.model.ikcon(firstTransform, originalConfig);  % Joint angles for first point
    secondConfig = URrobot.model.ikcon(secondTransform, firstConfig);   % Joint angles for second point
    
    % First: Move to the first point (fast movement)
    numStepsToFirstPoint = 50;  % Fewer steps for quicker movement to the first point
    qMatrixToFirstPoint = jtraj(originalConfig, firstConfig, numStepsToFirstPoint);
    
    % Animate the robot from its starting position to the first point
    disp('Moving to the first point with downward orientation...');
    for j = 1:size(qMatrixToFirstPoint, 1)
        URrobot.model.animate(qMatrixToFirstPoint(j, :));
        drawnow;
        pause(0.01);  % Adjust pause to control speed (faster)
    end
    
    % Second: Slowly move to the second point (slow movement)
    numStepsToSecondPoint = 150;  % More steps for slower movement to the second point
    qMatrixToSecondPoint = jtraj(firstConfig, secondConfig, numStepsToSecondPoint);
    
    disp('Slowly moving to the second point with downward orientation...');
    for j = 1:size(qMatrixToSecondPoint, 1)
        URrobot.model.animate(qMatrixToSecondPoint(j, :));
        drawnow;
        pause(0.02);  % Slower movement with more steps
    end
    
    % After reaching the second point, move back to the original position
    numStepsToReturn = 100;  % Set steps for returning to the original position
    qMatrixReturn = jtraj(secondConfig, originalConfig, numStepsToReturn);
    
    disp('Returning to the original position...');
    for j = 1:size(qMatrixReturn, 1)
        URrobot.model.animate(qMatrixReturn(j, :));
        drawnow;
        pause(0.02);  % Adjust for smooth return speed
    end
    
    disp('Returned to the original position.');
end

function MoveToDropOff(robot, swordPatch, swordVertices)
    % Initialize trackSword variable (always true for tracking)
    trackSword = true;  % Enable sword tracking for all points

    % Define waypoints for the robot
    first_point = [-1.1456, 0.98152, 0.9, 0, 0, 0];
    second_point =  [-1.1456, 0.98152, 0.7, 0, 0, -pi/2];
    third_point =  [-1.1456, 0.98152, 0.7, 0, 0, 0];
    Final_point =  [0, -0.8, 1.5, 0, 0, 0];

  
    % Move to the first point 
    disp('Moving to the dropoff...');
    robot.MoveTo(first_point, trackSword, swordPatch, swordVertices);
    % Wait for half a second

    % Move down to the second point (near
    disp('dropping off');
    robot.MoveTo(second_point, trackSword, swordPatch, swordVertices);
    trackSword = false;
    robot.MoveTo(third_point, trackSword, swordPatch, swordVertices);
    robot.MoveTo(Final_point, trackSword, swordPatch, swordVertices);

end

%%
hold off; % Release the plot hold


