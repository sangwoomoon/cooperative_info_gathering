function field = InitializeField(sim, RangeX, RangeY, RangeZ)

flagPlot = sim.flagPlot;

switch nargin
    case 3 % 2D
        field.boundary = [RangeX RangeY];
        field.length = ...
            [field.boundary(2)-field.boundary(1) field.boundary(4)-field.boundary(3)];
        field.buffer = 50; % for particle initalization
        field.bufferZone = [field.boundary(1)+field.buffer field.boundary(2)-field.buffer...
            field.boundary(3)+field.buffer field.boundary(4)-field.buffer];
        field.zoneLength = ...
            [field.bufferZone(2)-field.bufferZone(1) field.bufferZone(4)-field.bufferZone(3)];
        
        % field plotting setting
        if flagPlot
            figure(1)
            set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                'ylim',[field.boundary(3),field.boundary(4)])
            set(gca,'color','w')
            xlabel('East [m]'); ylabel('North [m]'); axis equal;
            title(sprintf('t = %.1f [sec]', 0))
        end
        
    case 4 % 3D
        field.boundary = [RangeX RangeY RangeZ];
        field.length = ...
            [field.boundary(2)-field.boundary(1) field.boundary(4)-field.boundary(3) field.boundary(6)-field.boundary(5)];
        field.buffer = 50; % for particle initalization
        field.bufferZone = [field.boundary(1)+field.buffer field.boundary(2)-field.buffer...
            field.boundary(3)+field.buffer field.boundary(4)-field.buffer...
            field.boundary(5)+field.buffer field.boundary(6)-field.buffer];
        field.zoneLength = ...
            [field.bufferZone(2)-field.bufferZone(1) field.bufferZone(4)-field.bufferZone(3) field.bufferZone(6)-field.bufferZone(5)];
        
        % field plotting setting
        if flagPlot
            figure(1)
            set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                'ylim',[field.boundary(3),field.boundary(4)],...
                'Zlim',[field.boundary(5),field.boundary(6)])
            set(gca,'color','w')
            xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]'); view(40,20);
            title(sprintf('t = %.1f [sec]', 0))
        end
end

end
    