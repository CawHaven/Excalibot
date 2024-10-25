classdef Camera
    properties
        position        % Camera position in 3D space [x, y, z]
        orientation     % Camera orientation (3x3 rotation matrix)
        axis_limits     % Axis limits for visualization
        env             % Reference to the Environment instance
        baseDistance    % Reference distance for calculating zoom
        displayFigure   % Handle to the figure for displaying captured views
        displayAxes     % Handle to the axes inside displayFigure
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
            obj.axis_limits = [-10 10 -10 10 -10 10]; % Default axis limits, can be customized
            obj.baseDistance = norm(position); % Initial reference distance from origin
            obj.displayFigure = []; % Initialize display figure as empty
            obj.displayAxes = []; % Initialize display axes as empty
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
        end

        % Capture the current view of the environment and update displayFigure
        function captureAndDisplay(obj, displayAxes)
            % Create an invisible figure to capture the environment view
            tempFigure = figure('Visible', 'off'); % Invisible figure
            envAxes = obj.env.envFigure.CurrentAxes; % Original environment axes
            
            % Copy environment axes to temporary figure and set it as active
            tempAxes = copyobj(envAxes, tempFigure);
            
            % Apply camera transformations to the temporary axes
            campos(tempAxes, obj.position);
            camup(tempAxes, obj.orientation(:, 3));
            camtarget(tempAxes, obj.orientation(:, 2)' + obj.position);
            
            % Apply zoom based on the calculated zoom factor
            zoomFactor = obj.calculateZoomFactor();
            camva(tempAxes, camva(envAxes) * zoomFactor); % Adjust the view angle
            
            % Capture the frame from the temporary axes
            frame = getframe(tempAxes); % Capture the frame data
            close(tempFigure); % Close the temporary figure

            % Display the captured view in displayFigure by overwriting the old image
            imshow(frame.cdata, 'Parent', displayAxes);
        end
    end
end