classdef Environment

    properties
    end

    methods
        function self = Environment()
            %% 1. Initialize Environment
            figure;
            hold on; 
            axis equal;
            grid on;
            view(3);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            xlim([-2.5, 2.5]); % Set X-axis limits meters
            ylim([-2.5, 2.5]); % Set Y-axis limits meter
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
            
            fencefile = 'Fencing.ply';
            fencePos = [0,0,0];
            PlaceObject(fencefile, fencePos);
            
            ABBPedestalfile = 'RobotPedestal.ply';
            ABBPedestalPos = [0,0,0];
            PlaceObject(ABBPedestalfile, ABBPedestalPos);
            
            CobotPedestalfile = 'URRobotPedestal.ply';
            CobotPedestalPos = [0,0,0];
            PlaceObject(CobotPedestalfile, CobotPedestalPos);
            
            waterfile = 'WaterBucket.ply';
            waterPos = [0,0,0];
            PlaceObject(waterfile, waterPos);
            
            furnacefile = 'Furnace.ply';
            furnacePos = [0,0,0];
            PlaceObject(furnacefile, furnacePos);
        end
    end
end