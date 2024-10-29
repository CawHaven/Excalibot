classdef Camera
    properties
        position        % Camera position in 3D space [x, y, z]
        orientation     % Camera orientation (3x3 rotation matrix)
        env             % Reference to the Environment instance
        baseDistance    % Reference distance for calculating zoom 
    end

    methods
        % Constructor
        function obj = Camera(env, position, orientation)
            if nargin < 3
                position = [0, 0, 0];
                orientation = eye(3);
            end
            obj.position = position;
            obj.orientation = orientation;
            obj.env = env;  % Store reference to the Environment
            obj.baseDistance = norm(position); % Initial reference distance from origin
        end

        % Set position of the camera
        function obj = setPosition(obj, newPosition)
            obj.position = newPosition;
        end

        % Set orientation of the camera
        function obj = setOrientation(obj, newOrientation)
            if all(size(newOrientation) == [3 3])
                obj.orientation = newOrientation;
            else
                error('Orientation must be a 3x3 rotation matrix');
            end
        end

        % Set position and orientation of the camera
        function obj = setPose(obj, newPosition, newOrientation)
            obj = obj.setPosition(newPosition);
            obj = obj.setOrientation(newOrientation);
        end

        % Calculate zoom factor based on the distance from a reference point
        function zoomFactor = calculateZoomFactor(obj)
            % Compute the current distance from the origin (or any reference point)
            currentDistance = norm(obj.position);
            % Set zoom factor as the ratio of the base distance to the current distance
            zoomFactor = obj.baseDistance / currentDistance;
            zoomFactor = zoomFactor * 0.55;
        end

        % Capture the current view of the environment and update displayFigure
        function captureAndDisplay(obj, tempAxes,envAxes)
            % Create an invisible figure to capture the environment view
           
            campos(tempAxes, obj.position);
            camup(tempAxes, obj.orientation(:, 3));
            camtarget(tempAxes, obj.orientation(:, 2)' + obj.position);
            
            % Apply zoom based on the calculated zoom factor
            zoomFactor = obj.calculateZoomFactor();
            camva(tempAxes, camva(envAxes) * zoomFactor); % Adjust the view angle     
        end
    end
end
