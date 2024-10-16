classdef Excalibot < RobotBaseClass
    %% Excalibot

    properties(Access = public)              
        plyFileNameStem = 'Excalibot';
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
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-360 360]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            
            self.model = SerialLink(link,'name',self.name);
        end
    end
end
