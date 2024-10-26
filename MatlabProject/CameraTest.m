% Create the environment
env = Environment();

% Create the camera with a reference to the environment
cam = Camera(env, [1, 1, 1], eye(3));
camPos = [1.15, 0.6, 1];

cam = cam.setPosition(camPos);
cam = cam.setOrientation(rotx(deg2rad(-90))); % Example using rotations
% Set a new position and orientation
% Check if displayFigure exists and is valid; otherwise, create it once
tempFigureHandle = figure('Visible', 'off'); % Invisible figure
FigureHandle = figure;
AxesHandle = axes('Parent', FigureHandle);
title(AxesHandle, 'Captured Camera View');

for i=0:10
    camPos = camPos - [0,0.05,0];
    cam = cam.setPosition(camPos);
    cam.captureAndDisplay(AxesHandle,tempFigureHandle);
end
