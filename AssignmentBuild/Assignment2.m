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

Sword = 'Sword.ply';
SwordPos = [0,0,0];
PlaceObject(Sword, SwordPos);

CobotPedestalfile = 'URRobotPedestal.ply';
CobotPedestalPos = [0,0,0];
PlaceObject(CobotPedestalfile, CobotPedestalPos);

waterfile = 'WaterBucket.ply';
waterPos = [0,0,0];
PlaceObject(waterfile, waterPos);

furnacefile = 'Furnace.ply';
furnacePos = [0,0,0];
PlaceObject(furnacefile, furnacePos);
%%
base = transl(-1.3134, -0.61995, 0.58111)
URrobot = UR5(base);
URrobot.model.fkine(URrobot.model.getpos);

baseTr = transl(-0.06748, 0.3819, 0)
robot = Excalibot(baseTr); % Initialize the robot
robot.model.fkine(robot.model.getpos);
hold on; % Keep the plot

disp('Press Enter to continue...');
pause;  % Wait for user input

% Move to the specified position
robot.MoveTo([1.25, 0.5, 0.75, 0, 0, 0]);


hold off; % Release the plot hold
