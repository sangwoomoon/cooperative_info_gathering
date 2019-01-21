function target = InitializeTarget(ID, sim)

flagPlot = sim.flagPlot;
targetID = sim.plot.targetID;
field = sim.field;
property = sim.flagTarget;

switch length(field.boundary)
    
    
    case 4 % 2D case
        
        switch property
            case 'Pos'
                % basic property setting
                target.id = ID;
                target.x = ...
                    [field.bufferZone(1)+rand()*field.zoneLength(1)...
                    field.bufferZone(3)+rand()*field.zoneLength(2)]'; % x_pos, y_pos
                target.hist.x = target.x;
                target.nState = length(target.x);
                
                %rotational moving
                %theta = 1; % deg/s
                %target.param.F = [cos(theta*D2R) -sin(theta*D2R); sin(theta*D2R) cos(theta*D2R)];
                target.param.F = eye(length(target.x));
                target.param.Q = zeros(target.nState); % certainly ideal
                
                % target display
                if flagPlot
                    
                    % plotting parameters setting: red, square
                    target.plot.clr = [1 rand() 0];
                    target.plot.marker = 'square';
                    target.plot.opaque = 0.1;
                    
                    % position plot setting
                    target.plot.pos = ...
                        plot(target.x(1),target.x(2),...
                        target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                    target.plot.id = ...
                        text(target.x(1),target.x(2),...
                        targetID(target.id));
                    
                    % trajectory plot setting
                    target.plot.path = animatedline(target.x(1),target.x(2),'color',target.plot.clr);
                    
                end
            case 'PosVel'
                
        end
        
        
    case 6 % 3D case
        
        
        switch property
            case 'Pos'
                % basic property setting
                target.id = ID;
                target.x = ...
                    [field.bufferZone(1)+rand()*field.zoneLength(1)...
                    field.bufferZone(3)+rand()*field.zoneLength(2)...
                    field.bufferZone(5)+rand()*field.zoneLength(3)]'; % x_pos, y_pos, z_pos
                target.hist.x = target.x;
                target.nState = length(target.x);
                
                %rotational moving
                %theta = 1; % deg/s
                %target.param.F = [cos(theta*D2R) -sin(theta*D2R); sin(theta*D2R) cos(theta*D2R)];
                target.param.F = eye(length(target.x));
                target.param.Q = zeros(target.nState); % certainly ideal
                
                % target display
                if flagPlot
                    
                    % plotting parameters setting: red, square
                    target.plot.clr = [1 rand() 0];
                    target.plot.marker = 'square';
                    target.plot.opaque = 0.1;
                    
                    % position plot setting
                    target.plot.pos = ...
                        plot3(target.x(1),target.x(2),target.x(3),...
                        target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                    target.plot.id = ...
                        text(target.x(1),target.x(2),target.x(3),...
                        targetID(target.id));
                    
                    % trajectory plot setting
                    target.plot.path = animatedline(target.x(1),target.x(2),target.x(3),'color',target.plot.clr);
                    
                end
            case 'PosVel'
                
        end
        
end

end