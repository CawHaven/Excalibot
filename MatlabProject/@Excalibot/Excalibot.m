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
            link(2).qlim = [-53 53]*pi/180;
            link(3).qlim = [-167 167]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
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
                self.model.animate(trajectoryclac(i, :));
                drawnow();
            end
        end
    end
end