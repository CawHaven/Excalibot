classdef Environment

    properties
        envFigure
    end

    methods
        function self = Environment()
            self.envFigure = figure;
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
        end
    end
end