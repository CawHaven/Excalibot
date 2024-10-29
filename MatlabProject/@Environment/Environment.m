classdef Environment

    properties
        envFigure
        envAxes
        sword
        hierarchicalBoundingBoxes % Hierarchical bounding boxes for each object

    end

    methods
        function self = Environment()
            self.envFigure = figure;
            self.envAxes = axes('Parent',self.envFigure);
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
            self.hierarchicalBoundingBoxes = struct();

            % Cobot Pedestal
            fencefile = 'Fencing.ply';
            fencePos = [0, 0, 0];
            PlaceObject(fencefile,fencePos);
            [fenceFaces, fenceVertices, ~] = plyread(fencefile, 'tri');
            fenceVerticesTransformed = self.transformVertices(fenceVertices, fencePos);
            self.hierarchicalBoundingBoxes.fence = self.createHierarchicalBoundingBoxes(fenceVerticesTransformed, 600);

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
            % Cobot Pedestal
            CobotPedestalfile = 'URRobotPedestal.ply';
            CobotPedestalPos = [0, 0, 0];
            PlaceObject(CobotPedestalfile,CobotPedestalPos);
            [pedestalFaces, pedestalVertices, ~] = plyread(CobotPedestalfile, 'tri');
            pedestalVerticesTransformed = self.transformVertices(pedestalVertices, CobotPedestalPos);
            self.hierarchicalBoundingBoxes.cobotPedestal = self.createHierarchicalBoundingBoxes(pedestalVerticesTransformed, 300);

            % Water Bucket
            waterfile = 'WaterBucket.ply';
            waterPos = [0.15, 0, 0];
            PlaceObject(waterfile,waterPos);
            [waterFaces, waterVertices, ~] = plyread(waterfile, 'tri');
            waterVerticesTransformed = self.transformVertices(waterVertices, waterPos);
            self.hierarchicalBoundingBoxes.waterBucket = self.createHierarchicalBoundingBoxes(waterVerticesTransformed, 300);

            % Furnace
            furnacefile = 'Furnace.ply';
            furnacePos = [0, 0, 0];
            PlaceObject(furnacefile,furnacePos);
            [furnaceFaces, furnaceVertices, ~] = plyread(furnacefile, 'tri');
            furnaceVerticesTransformed = self.transformVertices(furnaceVertices, furnacePos);
            self.hierarchicalBoundingBoxes.furnace = self.createHierarchicalBoundingBoxes(furnaceVerticesTransformed, 300);

            self.sword.patch = swordPatch;
            self.sword.vertices = swordVertices;
            %%
        end

        function boundingBoxes = createHierarchicalBoundingBoxes(self, vertices, minVertices)
            % Function to create a set of hierarchical bounding boxes for complex geometry
            % Inputs:
            % - vertices: Nx3 matrix of 3D points representing the vertices of the geometry
            % - minVertices: Minimum number of vertices required to stop dividing further
            % Output:
            % - boundingBoxes: Cell array of bounding box structs, each with min, max, and center

            % Base case: If the number of vertices is below the threshold, create a bounding box
            if size(vertices, 1) <= minVertices
                boundingBoxes = {self.createBoundingBox(vertices)};
                return;
            end

            % Calculate the bounding box of the current vertices
            bbox = self.createBoundingBox(vertices);

            % Determine the longest axis of the bounding box to split along
            extents = bbox.max - bbox.min;
            [~, splitAxis] = max(extents);

            % Find the median along the chosen axis to split the vertices
            medianValue = median(vertices(:, splitAxis));
            leftVertices = vertices(vertices(:, splitAxis) <= medianValue, :);
            rightVertices = vertices(vertices(:, splitAxis) > medianValue, :);

            % Recursively create bounding boxes for each half
            leftBoundingBoxes = self.createHierarchicalBoundingBoxes(leftVertices, minVertices);
            rightBoundingBoxes = self.createHierarchicalBoundingBoxes(rightVertices, minVertices);

            % Combine the bounding boxes from each half
            boundingBoxes = [leftBoundingBoxes, rightBoundingBoxes];
        end

        function bbox = createBoundingBox(~, vertices)
            % Sub-function to calculate an axis-aligned bounding box for a given set of vertices
            % Inputs:
            % - vertices: Nx3 matrix of vertices
            % Output:
            % - bbox: Struct containing min, max, and center of the bounding box

            minCoord = min(vertices);
            maxCoord = max(vertices);
            center = (minCoord + maxCoord) / 2;

            bbox = struct( ...
                'min', minCoord, ...
                'max', maxCoord, ...
                'center', center, ...
                'dimensions', maxCoord - minCoord ...
                );
        end

        function transformedVertices = transformVertices(~, vertices, position)
            % Apply translation to vertices
            transformMatrix = transl(position(1), position(2), position(3));
            transformedVertices = [vertices, ones(size(vertices, 1), 1)] * transformMatrix';
            transformedVertices = transformedVertices(:, 1:3); % Discard the homogeneous coordinate
        end

        function [isColliding, closestPoint] = checkPointCollision(self, point)
            % Function to check if a 3D point collides with any hierarchical bounding boxes
            % Input:
            % - point: a 1x3 vector representing the 3D coordinates of the point
            % Output:
            % - isColliding: boolean, true if collides with any bounding box
            % - closestPoint: 1x3 vector, closest non-colliding point if collision occurs

            isColliding = false;
            closestPoint = point;

            % Iterate through all objects in hierarchicalBoundingBoxes
            objectNames = fieldnames(self.hierarchicalBoundingBoxes);
            for objIdx = 1:numel(objectNames)
                % Retrieve all bounding boxes for the current object
                boundingBoxes = self.hierarchicalBoundingBoxes.(objectNames{objIdx});

                % Check each bounding box in the hierarchy
                for boxIdx = 1:numel(boundingBoxes)
                    bbox = boundingBoxes{boxIdx};

                    % Check if point is inside this bounding box
                    if self.isPointInBoundingBox(point, bbox)
                        isColliding = true;

                        % Calculate the closest non-colliding point by moving it to the nearest edge
                        closestPoint(1) = max(min(point(1), bbox.max(1)), bbox.min(1) - eps);
                        closestPoint(2) = max(min(point(2), bbox.max(2)), bbox.min(2) - eps);
                        closestPoint(3) = max(min(point(3), bbox.max(3)), bbox.min(3) - eps);

                        % Stop further checks if a collision is found
                        return;
                    end
                end
            end
        end

        function isInside = isPointInBoundingBox(~, point, bbox)
            % Helper function to check if a point is inside a bounding box
            isInside = ...
                (point(1) >= bbox.min(1) && point(1) <= bbox.max(1)) && ...
                (point(2) >= bbox.min(2) && point(2) <= bbox.max(2)) && ...
                (point(3) >= bbox.min(3) && point(3) <= bbox.max(3));
        end

    end
end