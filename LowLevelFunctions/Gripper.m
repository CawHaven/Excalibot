classdef Gripper < handle
    properties
        model;  % SerialLink model of the gripper
        workspace;  % Workspace limits for visualization
    end

    methods
        function self = TwoFingerGripper(workspace)
            self.workspace = workspace;
            self.CreateGripperModel();
            self.PlotAndColourGripper();
        end
    end
end