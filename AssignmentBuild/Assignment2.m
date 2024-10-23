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

disp('Task complete!');



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
    pause(0.5);  % Wait for half a second

    % Move down to the second point (on the sword)
    disp('Moving down to the second point (on the sword)...');
    robot.MoveTo(second_point, trackSword, swordPatch, swordVertices);
    pause(1);  % Wait for 1 second to simulate gripping the sword

    % Enable sword tracking and move back up
    disp('Gripping the sword and moving back up...');
    robot.EnableGripping();  % Enable sword tracking (gripping)
    robot.MoveTo(third_point, true, swordPatch, swordVertices);  % Start tracking the sword
    pause(0.5);  % Wait for half a second

    % Move to the third point
    disp('Moving to the third point...');
    robot.MoveTo(fourth_point, true, swordPatch, swordVertices);  % Continue tracking
    pause(0.5);  % Wait for half a second

    % Move to the final point
    disp('Moving to the final point...');
    robot.MoveTo(fifth_point, true, swordPatch, swordVertices);  % Continue tracking
    disp('Sword Heating Up');
end

%% Water function
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

%%
hold off; % Release the plot hold


