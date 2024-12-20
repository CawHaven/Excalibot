classdef Excalibot < RobotBaseClass
    %% Excalibot
    properties(Access = public)              
        plyFileNameStem = 'Excalibot';
        swordInitialTransform = [];  % Store the sword's initial transform when gripping starts
        isGripping = false;  % Flag to check if gripping has started
        gripperAPatch
        gripperBPatch
    end
    methods
%% Define robot Function 
function self = Excalibot(baseTr)
    self.CreateModel();
    if nargin < 1			
        baseTr = eye(4);				
    end
    self.model.base = self.model.base.T * baseTr;
    self.PlotAndColourRobot();
    % Get the current position of the robot's end-effector
    currentposition = self.model.fkine(self.model.getpos());  
    % Ensure that currentposition is a proper 4x4 matrix
    if isa(currentposition, 'SE3')
        currentposition = currentposition.T;  % Get the 4x4 transformation matrix
    end
    % Debugging: print the transformation matrix to verify it
    %disp('Current End-Effector Position (Transformation Matrix):');
    %disp(currentposition);
    
    % Check if currentposition is a valid 4x4 transformation matrix
    if size(currentposition, 1) == 4 && size(currentposition, 2) == 4
        % Extract the rotation matrix and convert to Euler angles
        rotationMatrix = currentposition(1:3, 1:3);
        % Ensure that rotation matrix elements are properly formatted
        %disp('Rotation Matrix:');
        %disp(rotationMatrix);   
        eulerAngles = rotm2eul(rotationMatrix, "ZYX");
    else
        error('Invalid transformation matrix from fkine.');
    end

    % Create gripper patches with current position and Euler angles
    [self.gripperAPatch, self.gripperBPatch] = self.createAndUpdateGripperModel(currentposition(1:3, 4), eulerAngles);
end

%% Create the robot model
        function CreateModel(self)   
            % Create Bot
            link(1) = Link('d',0.4,'a',0,'alpha',-pi/2,'offset',0);
            link(2) = Link('d',0,'a',1.05,'alpha',0,'offset',-pi/2);
            link(3) = Link('d',0.075,'a',0,'alpha',pi/2,'offset',pi/2);
            link(4) = Link('d',1,'a',0,'alpha',pi/2,'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'offset', 0);
            link(6) = Link('d',-0.18,'a',0,'alpha',0, 'offset',0);

            % Incorporate joint limits
            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-53 53]*pi/180;
            link(3).qlim = [-167 167]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
            link(5).qlim = [-98 98]*pi/180;
            link(6).qlim = [-260 100]*pi/180;

            self.model = SerialLink(link,'name',self.name);
        end

        %% Move to
        function MoveTo(self, position, trackSword, swordPatch, swordVertices)
    if length(position) ~= 6
        error('Input must be a 6-element array [x, y, z, rx, ry, rz]');
    end

    % Solve inverse kinematics to get the target joint angles
    inversekine = self.model.ikcon(transl(position(1), position(2), position(3)) * rpy2tr(position(4), position(5), position(6)));

    % Get current joint positions
    currentPos = self.model.getpos();

    % Ensure base (joint 1) moves in the shortest direction
    currentBaseAngle = currentPos(1);  % Base joint angle (joint 1)
    targetBaseAngle = inversekine(1);  % Target base joint angle (joint 1)

    % Calculate the angular difference (shortest path)
    baseMovement = targetBaseAngle - currentBaseAngle;
    baseMovement = mod(baseMovement + pi, 2*pi) - pi;  % Normalize to [-pi, pi] for the shortest path

    % Adjust the target base joint angle to follow the shortest path
    inversekine(1) = currentBaseAngle + baseMovement;

    % Generate a smooth trajectory between the current and target positions
    trajectoryclac = jtraj(currentPos, inversekine, 50);  % 50 steps for smooth interpolation
    % Animate the robot movement along the calculated trajectory
    for i = 1:size(trajectoryclac, 1)
        self.model.animate(trajectoryclac(i, :));

        % Update the gripper model's position
        self.updateGripperModel(self.gripperAPatch, self.gripperBPatch);

        % If sword tracking is enabled, update the sword mesh position
        if trackSword
            self.UpdateSwordMesh(swordPatch, swordVertices);
        end

        % Ensure the plot updates
        drawnow();
    end
  end

   %% Get End Effector Position
        function [t, R] = GetEndEffectorPosition(self)
            % Get current joint positions
            currentJointAngles = self.model.getpos();
    
            % Calculate the forward kinematics transformation matrix for the end-effector
            endEffectorTransform = self.model.fkine(currentJointAngles);  % Should return a SerialLink object
    
            % Ensure it's a 4x4 homogeneous transformation matrix
            if isa(endEffectorTransform, 'SE3')
                endEffectorTransform = endEffectorTransform.T;  % Get the transformation matrix
            end
    
            % Ensure it's the correct size (4x4)
            if size(endEffectorTransform, 1) == 4 && size(endEffectorTransform, 2) == 4
                % Extract the translation and rotation from the 4x4 transformation matrix
                t = endEffectorTransform(1:3, 4);  % 3x1 translation vector (x, y, z)
                R = endEffectorTransform(1:3, 1:3);  % 3x3 rotation matrix
            else
                error('The transformation matrix is not of the expected size 4x4.');
            end
        end

%% Update Sword Mesh Position
function UpdateSwordMesh(self, swordPatch, swordVertices)
    % Get current end-effector position and orientation
    [t, R] = self.GetEndEffectorPosition();
    
    % If gripping just started, store the initial sword transform
    if isempty(self.swordInitialTransform)
        self.swordInitialTransform = swordVertices;  % Save sword's current position when gripping starts
    end
    
    % Apply the relative transformation from the initial sword position
    swordVerticesTransformed = (R * self.swordInitialTransform.' + t).';  % Apply the relative transform

    % Lower the sword by 0.2m along the z-axis (subtract 0.2 from the z-component)
    lowerZTransform = transl(0, 0, -0.08);  % Create a transformation matrix to lower by 0.2m
    swordVerticesTransformed = [swordVerticesTransformed, ones(size(swordVerticesTransformed, 1), 1)] * lowerZTransform';  % Apply lowering transformation

    % Update the sword patch vertices
    set(swordPatch, 'Vertices', swordVerticesTransformed(:, 1:3));

    % Redraw the plot
    drawnow();
end

%% Enable Gripping
        function EnableGripping(self)
            self.isGripping = true;  % Set the gripping flag to true
        end
   %% Create and Update Gripper Model
        function [gripperAPatch, gripperBPatch] = createAndUpdateGripperModel(self, position, rotation)
            % Load PLY data for gripper components
            [GripperAFaces, GripperAVertices, ~] = plyread('GripperA.ply', 'tri');
            [GripperBFaces, GripperBVertices, ~] = plyread('GripperB.ply', 'tri');
            
            % Convert Euler angles to rotation matrix
            R = eul2rotm(rotation);  % Converts Euler angles (roll, pitch, yaw) to a rotation matrix
            t = position(:);         % Ensure position is a column vector
            
            % Apply transformation to gripper vertices
            GripperAVerticesTransformed = (R * GripperAVertices.' + t).';
            GripperBVerticesTransformed = (R * GripperBVertices.' + t).';
            
            % Create patch objects for each gripper component
          gripperAPatch = patch('Faces', GripperAFaces, 'Vertices', GripperAVerticesTransformed(:, 1:3), ...
            'FaceColor', 'blue', 'EdgeColor', 'none');
        gripperBPatch = patch('Faces', GripperBFaces, 'Vertices', GripperBVerticesTransformed(:, 1:3), ...
            'FaceColor', 'red', 'EdgeColor', 'none');
        end
%%
        function updateGripperModel(self, gripperAPatch, gripperBPatch)
            % Get current end-effector position and rotation
            [position, R] = self.GetEndEffectorPosition();
            
            % Update gripper A vertices
            newVerticesGripA = (R * get(gripperAPatch, 'Vertices')' + position).';
            set(gripperAPatch, 'Vertices', newVerticesGripA(:, 1:3));
            
            % Update gripper B vertices
            newVerticesGripB = (R * get(gripperBPatch, 'Vertices')' + position).';
            set(gripperBPatch, 'Vertices', newVerticesGripB(:, 1:3));
            
            % Redraw the plot
            drawnow();
        end

    end
end