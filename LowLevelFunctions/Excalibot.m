classdef Excalibot < RobotBaseClass
    %% Excalibot
    properties(Access = public)              
        plyFileNameStem = 'Excalibot'
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
            currentposition = self.model.fkine(self.model.getpos());
            eulerAngles = rotm2eul(currentposition(1:3, 1:3), "ZYX");
            [self.gripperAPatch, self.gripperBPatch] = createAndUpdateGripperModel(self, currentposition(1:3, 4), eulerAngles);
        end
%% Create the robot model
        function CreateModel(self)   
            % Create Bot
            link(1) = Link('d',0.4,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',1.05,'alpha',0,'qlim',deg2rad([-360 360]), 'offset',-pi/2);
            link(3) = Link('d',0.075,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',pi/2);
            link(4) = Link('d',1,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',-0.18,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset',0);
            
            % Incorporate joint limits
            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-53 53]*pi/180;
            link(3).qlim = [-167 167]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-98 98]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            
            self.model = SerialLink(link,'name',self.name);
        end
        %% Move to
        function MoveTo(self, position)
            if length(position) ~= 6
                error('Input must be a 6-element array [x, y, z, rx, ry, rz]');
            end
    
            inversekine = self.model.ikcon(transl(position(1), position(2), position(3)) * rpy2tr(position(4), position(5), position(6)));
            currentPos = self.model.getpos();
    
            trajectoryclac = jtraj(currentPos, inversekine, 50);
            for i = 1:size(trajectoryclac, 1)
                updateGripperModel(self.gripperAPatch, self.gripperBPatch);
                self.model.animate(trajectoryclac(i, :));
                drawnow();
            end
        end
        function [gripperAPatch, gripperBPatch] = createAndUpdateGripperModel(r, position, rotation)
            % Load PLY data for gripper components
            [GripperAFaces, GripperAVertices, GripperAData] = plyread('GripperA.ply', 'tri');
            [GripperBFaces, GripperBVertices, GripperBData] = plyread('GripperB.ply', 'tri');
            % Initial configuration for the robot's end effector
            initialConfig = zeros(1, r.model.n);  % Assuming zero configuration for all joints
            % Calculate the transformation matrix for the robot's end-effector
            endEffectorTransform = r.model.fkine(initialConfig).T;  % Transformation for the robot's end-effector
            % Create rotation matrices from the rotation angles
            R = eul2rotm(rotation);  % Converts Euler angles (roll, pitch, yaw) to a rotation matrix
            t = position(:);         % Ensure position is a column vector
            % Apply the end-effector transformation to each gripper component
            GripperAVerticesTransformed = (R * GripperAVertices.' + t).';
            GripperBVerticesTransformed = (R * GripperBVertices.' + t).';
            % Create patch objects for each gripper component
            gripperAPatch = patch('Faces', GripperAFaces, 'Vertices', GripperAVerticesTransformed(:, 1:3), ...
                'FaceVertexCData', [GripperAData.vertex.red, GripperAData.vertex.green, GripperAData.vertex.blue] / 255, ...
                'FaceColor', 'interp', 'EdgeColor', 'none');
            gripperBPatch = patch('Faces', GripperBFaces, 'Vertices', GripperBVerticesTransformed(:, 1:3), ...
                'FaceVertexCData', [GripperBData.vertex.red, GripperBData.vertex.green, GripperBData.vertex.blue] / 255, ...
                'FaceColor', 'interp', 'EdgeColor', 'none');
        end
        function updateGripperModel(gripperAPatch, gripperBPatch)
            % Update the position and rotation of the gripper
            position = self.model.fkine(self.model.getpos());
            rotation = rotm2eul(currentposition(1:3, 1:3), "ZYX");
            R = eul2rotm(rotation);  % Convert Euler angles to rotation matrix
            t = position(:);         % Ensure position is a column vector
            % Update vertices for Gripper A
            newVerticesGripA = (R * get(gripperAPatch, 'Vertices')' + t).';
            set(gripperAPatch, 'Vertices', newVerticesGripA(:, 1:3));
            % Update vertices for Gripper B
            newVerticesGripB = (R * get(gripperBPatch, 'Vertices')' + t).';
            set(gripperBPatch, 'Vertices', newVerticesGripB(:, 1:3));
            drawnow();
        end
        function R = eul2rotm(euler)
            % Converts roll, pitch, yaw (in radians) to a rotation matrix
            roll = euler(1);
            pitch = euler(2);
            yaw = euler(3);
            R_x = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
            R_y = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];
            R_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
            R = R_z * R_y * R_x;  % Combine the rotations
        end
    end
end