function filter = InitializeFilter(sim, iAgent, iTarget, xhat, Phat, Q, nPt)

filter.id = [iAgent, iTarget];

flagFilter = sim.flagFilter;
flagPlot = sim.flagPlot;
flagSensor = sim.flagSensor;

nTarget = sim.nTarget;

target = sim.target(iTarget);
field = sim.field;
sensor = sim.sensor(iAgent,iTarget);
dimension = length(target.x);

switch flagFilter
    case 'KF'
        
        filter.param.F = target.param.F;
        filter.param.R = sensor.param.R;
        filter.param.H = sensor.param.H;
        
        filter.xhat = xhat;
        filter.nState = length(xhat);
        
        switch dimension
            case 2 % 2D
                filter.param.Q = Q(1:2,1:2);
                filter.Phat = Phat(1:2,1:2);
            case 3 % 3D
                filter.param.Q = Q;
                filter.Phat = Phat;
        end
        
        % filter display
        if flagPlot
            
            % plotting parameter setting
            if nTarget == 1
                filter.plot.location.col = 1;
            else
                filter.plot.location.col = 2;
            end
            filter.plot.location.row = ceil(sim.nTarget/filter.plot.location.col);
            filter.plot.location.num = iTarget;
            
            % ellipsoid plot setting
            % AGENT 1 ONLY VISUALIZES PARTICLE BECAUSE OF HUGE PLOTTING
            % SPACE!
            if iAgent == 1
                figure(1+iAgent)
                subplot(filter.plot.location.col,...
                    filter.plot.location.row,...
                    filter.plot.location.num),
                
                switch dimension
                    case 2 % 2D

                        set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                            'ylim',[field.boundary(3),field.boundary(4)])
                        xlabel('East [m]'); ylabel('North [m]'); axis equal; hold on;
                        
                        % actual target plot setting for comparison
                        filter.plot.targetPos = ...
                            plot(target.x(1),target.x(2),...
                            target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                        filter.plot.targetId = ...
                            text(target.x(1),target.x(2),...
                            sim.plot.targetID(iTarget));
                        
                    case 3 % 3D
                        
                        set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                            'ylim',[field.boundary(3),field.boundary(4)],...
                            'zlim',[field.boundary(5),field.boundary(6)])
                        xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]'); hold on; view(40,20);
                        
                        % actual target plot setting for comparison
                        filter.plot.targetPos = ...
                            plot3(target.x(1),target.x(2),target.x(3),...
                            target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                        filter.plot.targetId = ...
                            text(target.x(1),target.x(2),target.x(3),...
                            sim.plot.targetID(iTarget));                        
                        
                end
                
                % ellipsoid plot
                filter.plot.ellipsoid = PlotGaussianEllipsoid(filter.xhat', filter.Phat);
                
            end
        end
        
    case 'EKF'
        
    case 'UKF'
        
    case 'PF'
        
        filter.id = [iAgent, iTarget];
        filter.nPt = nPt;
        filter.w = ones(1,filter.nPt)./filter.nPt;
        
        if iAgent == 1
            for iPt = 1 : filter.nPt
                
                switch dimension
                    case 2 % 2D
                        filter.pt(:,iPt) = mvnrnd(xhat,Phat(1:2,1:2))';
%                         filter.pt(:,iPt) = ...
%                             [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2)]';
                    case 3 % 3D
                        filter.pt(:,iPt) = mvnrnd(xhat,Phat)';
%                         filter.pt(:,iPt) = ...
%                             [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2) field.bufferZone(5)+rand()*field.zoneLength(3)]';
                end
                    
            end
        else
            filter.pt = sim.filter(1,iTarget).pt; % in order to make the same initial condition
        end
        
        filter.param.F = target.param.F;
        switch dimension
            case 2
                filter.param.Q = Q(1:2,1:2);
            case 3
                filter.param.Q = Q;
        end
        
        switch flagSensor
            case 'PosLinear'
                filter.param.R = sensor.param.R;
            case 'range_bear'
                filter.param.R = sensor.param.R;
        end
        
        filter.param.field = field;
        filter.nState = target.nState;
        
        filter.xhat = (filter.w*filter.pt')';
        
        filter.hist.pt(:,:,1) = filter.pt;
        filter.hist.w(:,:,1) = filter.w;
        filter.hist.xhat(:,1) = filter.xhat;
        
        % filter display
        if flagPlot
            
            % plotting parameter setting
            if nTarget == 1
                filter.plot.location.col = 1;
            else
                filter.plot.location.col = 2;
            end
            filter.plot.location.row = ceil(sim.nTarget/filter.plot.location.col);
            filter.plot.location.num = iTarget;
            
            filter.plot.clr = 'magenta';
            filter.plot.marker = '.';
            filter.plot.size = 2;
            
            % particle scatter plot setting
            % AGENT 1 ONLY VISUALIZES PARTICLE BECAUSE OF HUGE PLOTTING
            % SPACE!
            if iAgent == 1
                figure(1+iAgent)
                subplot(filter.plot.location.col,...
                    filter.plot.location.row,...
                    filter.plot.location.num),
                
                switch dimension
                    case 2 % 2D
                        
                        set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                            'ylim',[field.boundary(3),field.boundary(4)])
                        xlabel('East [m]'); ylabel('North [m]'); axis equal; hold on;
                        
                        % actual target plot setting for comparison
                        filter.plot.targetPos = ...
                            plot(target.x(1),target.x(2),...
                            target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                        filter.plot.targetId = ...
                            text(target.x(1),target.x(2),...
                            sim.plot.targetID(iTarget));
                        
                        % particle plot
                        filter.plot.pt = ...
                            scatter(filter.pt(1,:),filter.pt(2,:),10,filter.w,'filled');
                        
                    case 3 % 3D

                        set(gca,'xlim',[field.boundary(1),field.boundary(2)],...
                            'ylim',[field.boundary(3),field.boundary(4)],...
                            'zlim',[field.boundary(5),field.boundary(6)])
                        xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]'); hold on; view(40,20);
                        
                        % actual target plot setting for comparison
                        filter.plot.targetPos = ...
                            plot3(target.x(1),target.x(2),target.x(3),...
                            target.plot.marker,'LineWidth',2,'color',target.plot.clr);
                        filter.plot.targetId = ...
                            text(target.x(1),target.x(2),target.x(3),...
                            sim.plot.targetID(iTarget));
                        
                        % particle plot
                        filter.plot.pt = ...
                            scatter3(filter.pt(1,:),filter.pt(2,:),filter.pt(3,:),10,filter.w,'filled');
                        
                end
                
            end
            
        end
        
end

% actual entropy and its reduction initialization
filter.Hbefore = nan;
filter.Hafter = nan;
filter.I = nan;

filter.hist.Hbefore(:,1) = filter.Hbefore;
filter.hist.Hafter(:,1) = filter.Hafter;
filter.hist.I(:,1) = filter.I;

    
end