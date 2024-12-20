%% Main Script: Hand-Eye Coordination with Hand-Eye Calibration
clc;
clf;
clear;

%% Robot and Environment Initialisation
robot = UR3e();
hold on;

% Set up virtual objects (table, bricks, etc.)
brick = PlaceObjectV2('Brick.ply', [0.5, 0, 0], [0, 0, 0]);

%% Camera Initialisation
[cameraParams, camPosition, camOrientation] = initCamera();

%% Hand-Eye Calibration
% Construct multiple robot and camera poses
robotPoses = cat(3, ...
    [0.7071, -0.7071, 0, 0.1000;
     0.7071,  0.7071, 0, -0.1000;
     0,        0,     1, -0.2000;
     0,        0,     0, 1], ...
    [1, 0, 0, 0.2;
     0, 1, 0, 0.1;
     0, 0, 1, -0.15;
     0, 0, 0, 1]);

cameraPoses = cat(3, ...
    [0.7571, -0.6533, -0.0048, 0.0653;
     0.6124,  0.7071,  0.3536, -0.1500;
    -0.2276, -0.2706,  0.9354, -0.0271;
     0,        0,      0, 1], ...
    [0.8660, -0.5000, 0, 0.15;
     0.5000,  0.8660, 0, -0.05;
     0,        0,     1, -0.12;
     0,        0,     0, 1]);

% Use first pose for calibration
A_rel = robotPoses(:, :, 1);
B_rel = cameraPoses(:, :, 1);

T_camera_to_robot = handEyeCalibration(A_rel, B_rel);

%% Main Control Loop - Hand-Eye Coordination
n_points = 90;
target_brick_position = [0.5, 0, 0.2]; % Example target position for the robot to reach

for i = 1:5
    %% Simulate Camera View
    % Project 3D point (e.g., the brick position) into 2D image plane
    imagePoints = projectObjectToCamera(target_brick_position, camPosition, camOrientation, cameraParams);
    
    % Display the detected brick position in 2D image plane
    disp("Detected Brick at Image Coordinates: ");
    disp(imagePoints);
    
    %% Check Accuracy of Detected Coordinates
    isDetectedCorrectly = checkDetectionAccuracy(imagePoints, cameraParams, target_brick_position);
    if isDetectedCorrectly
        disp('Detected coordinates are correct.');
    else
        disp('Detected coordinates are incorrect.');
    end
    
    %% Hand-Eye Coordination - Move Robot to Align with the Brick
    % Apply hand-eye calibration to convert camera coordinates to robot coordinates
    target_in_robot_frame = T_camera_to_robot * [target_brick_position, 1]'
    target_brick_in_robot = target_in_robot_frame(1:3)'
    
    % Move robot to the transformed position
    pos = move_bot(robot, target_brick_position, n_points);
    for j = 1:n_points
        robot.model.animate(pos(j, :));
        drawnow();
    end
end

%% Function Definitions

% 1. Camera Initialisation
function [cameraParams, camPosition, camOrientation] = initCamera()
    % Define camera intrinsics
    focalLength = [800, 800];   % Focal length in pixels
    principalPoint = [320, 240]; % Principal point in pixels
    imageSize = [640, 480];      % Image size [rows, columns]
    
    % Create camera parameters object with correct intrinsic matrix format
    intrinsicMatrix = [focalLength(1), 0, principalPoint(1); 
                       0, focalLength(2), principalPoint(2); 
                       0, 0, 1];
    
    cameraParams = cameraParameters('IntrinsicMatrix', intrinsicMatrix', 'ImageSize', imageSize);
    
    % Define camera extrinsics (position and orientation)
    camPosition = [0.8, 0, 0.5]; % Camera position in 3D world
    camOrientation = [0, 0, 0];  % Euler angles (radians) for camera orientation
end

function T_camera_to_robot = handEyeCalibration(A_rel, B_rel)
    % Extract the rotation and translation components from A_rel and B_rel
    R_A = A_rel(1:3, 1:3);  % Rotation of robot relative to some fixed frame
    t_A = A_rel(1:3, 4);    % Translation of robot relative to some fixed frame
    
    R_B = B_rel(1:3, 1:3);  % Rotation of camera relative to some fixed frame
    t_B = B_rel(1:3, 4);    % Translation of camera relative to some fixed frame
    
    % Solve for the rotation part using least squares
    A_rot = kron(eye(3), R_A);
    B_rot = R_B(:);
    
    % Solve for the rotation matrix X_rot
    X_vec = A_rot \ B_rot;  % Solve AX = B for the rotation part
    
    % Reshape the solution into a 3x3 matrix
    if numel(X_vec) ~= 9
        error('The computed X_vec does not have 9 elements.');
    end
    X_rot = reshape(X_vec, 3, 3);
    
    % Ensure that X_rot is a proper rotation matrix
    [U, ~, V] = svd(X_rot);
    X_rot = U * V';  % Orthogonalize the rotation matrix
    
    % Solve for the translation part
    A_trans = eye(3) - X_rot;
    X_trans = A_trans \ (t_B - X_rot * t_A);  % Solve for translation
    
    % Construct the final transformation matrix T_camera_to_robot
    T_camera_to_robot = eye(4);
    T_camera_to_robot(1:3, 1:3) = X_rot;  % Insert the rotation part
    T_camera_to_robot(1:3, 4) = X_trans;  % Insert the translation part
    
    disp('Hand-eye calibration successful.');
    disp(T_camera_to_robot);
end

% 3. Project Object from 3D to 2D Image Plane
function imagePoints = projectObjectToCamera(objectPosition, camPosition, camOrientation, cameraParams)
    % Convert camera orientation (Euler angles) to rotation matrix
    R = eul2rotm(camOrientation);
    
    % Convert camera position to translation vector
    t = camPosition';
    
    % Project the 3D point (objectPosition) to the 2D image plane using camera parameters
    imagePoints = projectPoints(objectPosition, R, t, cameraParams.IntrinsicMatrix);
end

function pixelCoord = projectPoints(objectPosition, R, t, intrinsicMatrix)
    % Ensure objectPosition is a column vector
    objectPosition = objectPosition(:);  % Convert to column vector

    % Convert 3D object position to camera coordinates
    cameraCoord = R * objectPosition + t; % This should give you a 3x1 vector

    % Check dimensions of cameraCoord
    if size(cameraCoord, 1) ~= 3
        error('cameraCoord must have 3 rows for x, y, z coordinates.');
    end
    
    % Project onto image plane by converting to homogeneous coordinates
    cameraCoord_homogeneous = cameraCoord; % Now it's a 4x1 vector for homogeneous coordinates

    % Use the intrinsic matrix to project the 3D point onto the 2D image plane
    pixelCoord_homogeneous = intrinsicMatrix * cameraCoord_homogeneous; % This should yield a 3x1 vector

    % Normalize pixel coordinates (convert from homogeneous to Cartesian)
    pixelCoord = pixelCoord_homogeneous(1:2) ./ pixelCoord_homogeneous(3); % Ensure it works correctly
end

% 4. Move the Robot to a Target Position
function positions = move_bot(robot, relative_pos, n_points)
    % Use inverse kinematics to move the robot to the target end position
    end_pos =  relative_pos;
    positions = jtraj(robot.model.getpos, robot.model.ikcon(transl(end_pos(1), end_pos(2), end_pos(3))), n_points);
end

function [ mesh_h ] = PlaceObjectV2( name , locations, rotations )
    % Read the PLY file
    [f, v, data] = plyread(name, 'tri');
    
    % Default values
    if nargin < 2
        locations = [0, 0, 0];
    end
    if nargin < 3
        rotations = zeros(size(locations)); % No rotation if not specified
    end
    
    % Default color
    defaultColor = [0.8, 0.8, 0.8];
    
    % Check and scale colors
    if isfield(data, 'vertex') && isfield(data.vertex, 'red')
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    elseif isfield(data, 'face') && isfield(data.face, 'red')
        vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;
    else
        vertexColours = defaultColor; % Default to gray if no color information
    end
    
    % Place object at defined locations and rotations
    mesh_h = zeros(size(locations, 1), 1);
    for i = 1:size(locations, 1)
        % Extract current location and rotation
        loc = locations(i, :);
        rot = rotations(i, :);
        
        % Compute rotation matrices
        Rx = [1, 0, 0;
              0, cos(rot(1)), -sin(rot(1));
              0, sin(rot(1)), cos(rot(1))];
          
        Ry = [cos(rot(2)), 0, sin(rot(2));
              0, 1, 0;
              -sin(rot(2)), 0, cos(rot(2))];
          
        Rz = [cos(rot(3)), -sin(rot(3)), 0;
              sin(rot(3)), cos(rot(3)), 0;
              0, 0, 1];
        
        % Combine rotation matrices
        R = Rz * Ry * Rx;
        
        % Apply rotation to vertices
        v_rot = (R * v')';
        
        % Apply mirroring if specified
        v_rot(:,1) = -v_rot(:,1);
        
        % Translate vertices
        v_trans = v_rot + loc;
        
        % Plot the Trisurf
        mesh_h(i) = trisurf(f, v_trans(:,1), v_trans(:,2), v_trans(:,3), ...
            'FaceVertexCData', vertexColours, 'EdgeColor', 'none', 'EdgeLighting', 'none');
    end
end

% 6. Check the Accuracy of Detected Coordinates
function isDetectedCorrectly = checkDetectionAccuracy(detectedPoints, cameraParams, actualPosition)
    % Calculate the expected image coordinates from the actual 3D position
    expectedImagePoints = projectPoints(actualPosition, eye(3), [0; 0; 0], cameraParams.IntrinsicMatrix);
    
    % Define a tolerance for detection accuracy
    tolerance = 10;  % in pixels
    
    % Compare detected points with expected points
    distance = norm(detectedPoints - expectedImagePoints);
    isDetectedCorrectly = distance <= tolerance;  % True if within tolerance
end
