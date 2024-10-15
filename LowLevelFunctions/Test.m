clc;
clf;
clear;

a = PlaceObject('ExcalibotLink0.ply', [0,0,0]);
%b = PlaceObject('Swivel.ply', [0,0,0]);
%c = PlaceObject('FirstArm.ply', [0,0,0]);
%d = PlaceObject('FirstKnuckle.ply', [0,0,0]);
%e = PlaceObject('SecondArm.ply', [0,0,0]);
%f = PlaceObject('SecondKnuckle.ply', [0,0,0]);
%g = PlaceObject('Endeffector.ply', [0,0,0]);
%h = PlaceObject('GripperA.ply', [0,0,0]);
%i = PlaceObject('GripperB.ply', [0,0,0]);

% Set up the axes
axis equal;
grid on;

% Position the camera for better view
view(3);
camlight('headlight');

% Define the position of the light source
lightPos = [1, 1, 1]; % You can adjust these values

% Create a light source
light('Position', lightPos, 'Style', 'local', 'Color', [1 1 1]);

% Dim the light intensity
set(gcf, 'Color', 'k'); % Set figure background color to black
set(gca, 'Color', 'k'); % Set axes background color to black
lighting gouraud; % Use Gouraud lighting
material dull; % Use a dull material for the object

% Optionally adjust light properties
lightHandle = light('Position', lightPos, 'Style', 'local');
set(lightHandle, 'Color', [1 1 1] * 0.2); % Dim the light intensity (scale down)

% Display the object
