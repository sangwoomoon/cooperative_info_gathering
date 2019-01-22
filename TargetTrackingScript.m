
% Target Localization Problem Script
% Particle Method based Mutual Information
%
% two agents in the problem: 1st - receiving info / 2nd - tracking and
% measure information
%
%
% X(t+1) ~ P_ta(X(t+1)|X(t))
% Y(t)   ~ P_se(Y(t)|X(t);s(t))
%
% - coded by Sangwoo Moon
% - Created:      4/ 3/2018
% - 1st revision: 4/18/2018
% - 2nd revision: 6/25/2018
% - 3rd revision: 9/ 5/2018
% - 4th revision:12/ 8/2018
% - 5th revision: 1/17/2019
%
%   X(t+1) = X(t) + W                               : 2D-static Linear Gaussian
%   Y(t) = {0,1} with respect to Sensing Region     : 2D on the ground, circle region for target detection
%   s(t+1) = f(s(t),u(t))                           : u(t) = [0 -omega +omega]


close all;
clear;
% clc;
format compact;
hold on;

nSim = 1; % for Monte-Carlo approach or method comparison

RandSeed = rng;

for iSim = 1:nSim
    
    %-------------------------------
    %   sim setting
    %-------------------------------

    rng(RandSeed);
    
    %----------------------
    % simulation structure
    sim(iSim) = InitializeSim(   2,       1,     'MI',       1,       'uniform',        0,         1,     'Pos',  'unicycle', 'PosLinear',   'KF'    ); 
                            % nAgent | nTarget | flagDM | flagComm | flagPdfCompute | flagLog | flagPlot | target |  agent     | sensor   | filter
    
    % flagDM         ||   'random': random decision | 'MI': mutual information-based decision | 'mean': particle mean following
    % flagComm       ||   0: perfect communication | 1: imperfect communication and communication awareness
    % flagPdfCompute ||   'uniform': uniformly discretized domain | 'cylinder': cylinder based computation w.r.t particle set
    % flagLog        ||   0: skip logging | 1: log data
    % flagPlot       ||   flag for the display of trajectories and particles evolution
    % target         ||   'Pos': position only | 'PosVel': position and velocity
    % sensor         ||   'linear', 'range_bear', 'detection'
    %----------------------
    
    %----------------------
    % clock structure
    sim(iSim).clock = InitializeClock(   2  ,   1  );
                                       % nt  |  dt
    %----------------------
    
    %----------------------
    % field structure
    sim(iSim).field = InitializeField(sim(iSim), [-300 300],[-300 300]); % overloaded by the number of boundaries (2D)     
    % sim(iSim).field = InitializeField(sim(iSim), [-300 300],[-300 300],[-300,300]); % overloaded by the number of boundaries (3D)     
    %----------------------
    
    %----------------------
    % target structure
    for iTarget = 1:sim(iSim).nTarget
        sim(iSim).target(iTarget) = InitializeTarget(iTarget, sim(iSim));
    end
    %----------------------
    
    %----------------------
    % agent structure

    % parameters fleet of agent for initial positioning
    for iAgent = 1:sim(iSim).nAgent
        sim(iSim).agent(iAgent) = InitializeAgent(iAgent, sim(iSim), 10);
    end
    %----------------------
    
    %----------------------
    % sensor structure
    for iAgent = 1:sim(iSim).nAgent
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).sensor(iAgent,iTarget) = InitializeSensor(sim(iSim),iAgent,iTarget,   40,    0.9,  sim(iSim).agent(iAgent), sim(iSim).target(iTarget), diag([20^2,20^2,20^2]'));
                                                                                          % range | beta |                                                           R
        end  
    end
    %----------------------
    
    %----------------------
    % communication structure
    for iAgent = 1:sim(iSim).nAgent
        sim(iSim).comm(iAgent) = InitializeCommunication(iAgent,sim);
    end
    %----------------------
    
    %----------------------
    % filter structure
    for iAgent = 1:sim(iSim).nAgent
        for iTarget = 1:sim(iSim).nTarget
            % nPt = floor(10^(0.15*(iSim+6)));
            xhat = zeros(length(sim(iSim).target(iTarget).x),1);
            Phat = diag([50^2,50^2,50^2]);
            sim(iSim).filter(iAgent,iTarget) = InitializeFilter(sim(iSim),iAgent,iTarget,  xhat,  Phat,   diag([18^2,18^2,18^2]), 500);
                                                                                        %  xhat | Phat   |            Q         | nPt
        end
    end
    %----------------------
    
    %----------------------
    % planner structure
    for iAgent = 1:sim(iSim).nAgent
        sim(iSim).planner(iAgent) = InitializePlanner(iAgent,sim, 3,  5,  500 );
                                                               % dt | nT | nPt                
    end
    %----------------------

    
    %% ---------------------------------
    % Sim Operation
    %-----------------------------------
    
    for iClock = 1:sim(iSim).clock.nt
        
        %-----------------------------------
        % PF-based Mutual information Computation and decision-making
        %-----------------------------------
        
        % compute future information with respect to action profiles
        % distributed scheme to each agent:
        % COMPUTED INFORMATION IS DIFFERENT WITH RESPECT TO AGENT
        for iAgent = 1:sim(iSim).nAgent
            
            % decision making procedure
            switch sim(iSim).flagDM
                
                % random decision
                case 'random'
                    
                    % check the possible action command using geofence
                    bAction = nan(1,sim(iSim).planner(iAgent).actionNum);
                    for iAction = 1:sim(iSim).planner(iAgent).actionNum
                       
                        state = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).actionSet(iAction),sim(iSim).clock.dt);
                        if (state(1) > sim(iSim).field.bufferZone(1) && state(1) < sim(iSim).field.bufferZone(2)) ...
                                && (state(2) > sim(iSim).field.bufferZone(3) && state(2) < sim(iSim).field.bufferZone(4)) % inside geofence
                            bAction(iAction) = 1;
                        else
                            bAction(iAction) = 0;
                        end
                        
                    end
                    
                    % AD-HOC: when the agent is close to the geofence so that all
                    % cost candidates are infinity, then go to the origin.
                    if sum(bAction) == 0 
                        sim(iSim).planner(iAgent).actIdx = MoveToPoint([0 0], sim(iSim).agent(iAgent).s);
                    else
                        sim(iSim).planner(iAgent).actIdx = ceil(rand()*sim(iSim).planner(iAgent).actionNum);
                    end
                    sim(iSim).planner(iAgent).input = sim(iSim).planner(iAgent).actionSet(:,sim(iSim).planner(iAgent).actIdx);
                    
                % follow mean of a single target
                case 'mean'
                    
                    iTarget = 1;
                    sim(iSim).planner(iAgent).actIdx = MoveToPoint(sim(iSim).planner(iAgent).PTset(iTarget).xhat, sim(iSim).agent(iAgent).s);
                    sim(iSim).planner(iAgent).input = sim(iSim).planner(iAgent).actionSet(:,sim(iSim).planner(iAgent).actIdx);
                    
                    
                % mutual information-based optimization
                case 'MI'
                    
                    for iAction = 1 : sim(iSim).planner(iAgent).actionSetNum
                        
                        % check whether decision has feasibility in terms of geofence
                        state = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).actionSet(iAction),sim(iSim).clock.dt);
                        if (state(1) > sim(iSim).field.bufferZone(1) && state(1) < sim(iSim).field.bufferZone(2)) ...
                                && (state(2) > sim(iSim).field.bufferZone(3) && state(2) < sim(iSim).field.bufferZone(4)) % inside geofence
                            
                            if ~sim(iSim).flagInfoCom
                                % Ryan's approach-based Mutual Information computation: Measurement sampling-based
                                [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction),...
                                    sim(iSim).planner(iAgent).candidate.HbeforeRef(:,iAction),sim(iSim).planner(iAgent).candidate.HafterRef(:,iAction),sim(iSim).planner(iAgent).candidate.IRef(iAction)] = ...
                                    ComputeInformation(sim(iSim).planner(iAgent),agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,sim(iSim),iAction,iClock);
                            elseif sim(iSim).flagInfoCom
                                % Mutual Information computation: Consider all future measurements
                                % consider communicaiton awareness
                                [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction),...
                                    sim(iSim).planner(iAgent).candidate.HbeforeRef(:,iAction),sim(iSim).planner(iAgent).candidate.HafterRef(:,iAction),sim(iSim).planner(iAgent).candidate.IRef(iAction)] = ...
                                    ComputeInformationMeasConsider(iAgent,iAction,iClock,sim(iSim));
%                                     ComputeInformationMeasConsider(sim(iSim).planner(iAgent),sim(iSim).agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,...
%                                     sim(iSim).flagDisp,sim(iSim).flagComm,sim(iSim).flagPdfCompute,...
%                                     iAction,iClock,sim(iSim).agent(iAgent).id);
                            end
                            
                        else % out of geofence
                            sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction) = inf;
                            sim(iSim).planner(iAgent).candidate.Hafter(:,iAction) = inf;
                            sim(iSim).planner(iAgent).candidate.I(iAction) = inf;
                        end
                    end
                    
                    % decision making: maximize mutual information
                    
                    % AD-HOC: when the agent is close to the geofence so that all
                    % cost candidates are infinity, then go to the origin.
                    if min(sim(iSim).planner(iAgent).candidate.I) == inf
                        sim(iSim).planner(iAgent).actIdx = MoveToPoint([0 0], sim(iSim).agent(iAgent).s);
                    else
                        [~,sim(iSim).planner(iAgent).actIdx] = max(sim(iSim).planner(iAgent).candidate.I);
                    end
                    sim(iSim).planner(iAgent).input = sim(iSim).planner(iAgent).actionSet(:,sim(iSim).planner(iAgent).actIdx);
                    
                    sim(iSim).planner(iAgent).I = sim(iSim).planner(iAgent).candidate.I(sim(iSim).planner(iAgent).actIdx);
                    sim(iSim).planner(iAgent).Hbefore = sim(iSim).planner(iAgent).candidate.Hbefore(:,sim(iSim).planner(iAgent).actIdx);
                    sim(iSim).planner(iAgent).Hafter = sim(iSim).planner(iAgent).candidate.Hafter(:,sim(iSim).planner(iAgent).actIdx);
                    
                    sim(iSim).planner(iAgent).IRef = sim(iSim).planner(iAgent).candidate.IRef(sim(iSim).planner(iAgent).actIdx);
                    sim(iSim).planner(iAgent).HbeforeRef = sim(iSim).planner(iAgent).candidate.HbeforeRef(:,sim(iSim).planner(iAgent).actIdx);
                    sim(iSim).planner(iAgent).HafterRef = sim(iSim).planner(iAgent).candidate.HafterRef(:,sim(iSim).planner(iAgent).actIdx);
                    
                    % add computed information to analyze Monte-Carlo based
                    % approach
                    sim(iSim).planner(iAgent).Isum = sim(iSim).planner(iAgent).Isum + sim(iSim).planner(iAgent).I;
                    sim(iSim).planner(iAgent).HbeforeSum = sim(iSim).planner(iAgent).HbeforeSum + sim(iSim).planner(iAgent).Hbefore;
                    sim(iSim).planner(iAgent).HafterSum = sim(iSim).planner(iAgent).HafterSum + sim(iSim).planner(iAgent).Hafter;
                    
            end
            
        end
            
        
        
        
        %-----------------------------------
        % Actual Agent-Target Dynamics and Measurement
        %-----------------------------------
        
        %-----------------------------------
        % target moving
        for iTarget = 1:sim(iSim).nTarget
            % target dynamics/store data
            sim(iSim).target(iTarget).x = UpdateTargetState(sim(iSim).target(iTarget).x,sim(iSim).target(iTarget).param,sim(iSim).clock.dt);
            sim(iSim).target(iTarget).hist.x(:,iClock+1) = sim(iSim).target(iTarget).x;
            
            % update plot
            if sim(iSim).flagPlot
                set(sim(iSim).target(iTarget).plot.pos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
                set(sim(iSim).target(iTarget).plot.id,'position',[sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
                addpoints(sim(iSim).target(iTarget).plot.path,sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)); 
            end
        end
        %-----------------------------------
        
        for iAgent = 1:sim(iSim).nAgent
            
            %-----------------------------------
            % agent moving
            % agent dynamics/store data
            sim(iSim).agent(iAgent).s = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).input(1),sim(iSim).clock.dt);
            sim(iSim).agent(iAgent).hist.s(:,iClock+1) = sim(iSim).agent(iAgent).s;
            
            % update plot 
            if sim(iSim).flagPlot
                set(sim(iSim).agent(iAgent).plot.pos,'Xdata',sim(iSim).agent(iAgent).s(1),'Ydata',sim(iSim).agent(iAgent).s(2));
                set(sim(iSim).agent(iAgent).plot.id,'position',[sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2)]);
                addpoints(sim(iSim).agent(iAgent).plot.path,sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2));
                set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                    'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
            end
            %-----------------------------------
        
            
            %-----------------------------------
            % take measurement
            sim(iSim).sensor(iAgent,1).plot.bDetect = 0;
            for iTarget = 1:sim(iSim).nTarget
                sim(iSim).sensor(iAgent,iTarget).y = ...
                    TakeMeasurement(sim(iSim).target(iTarget).x,sim(iSim).agent(iAgent).s,sim(iSim).sensor(iAgent,iTarget).param,sim(iSim).flagSensor);
                sim(iSim).sensor(iAgent,iTarget).hist.y(:,iClock+1) = sim(iSim).sensor(iAgent,iTarget).y;
                
                if sim(iSim).sensor(iAgent,iTarget).y == 1
                    sim(iSim).sensor(iAgent,1).plot.bDetect = 1;
                end
            end
            sim(iSim).sensor(iAgent,1).plot.hist.bDetect(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.bDetect;
            
            % update plot
            if sim(iSim).flagPlot
                [sim(iSim).sensor(iAgent,1).plot.data.x,sim(iSim).sensor(iAgent,1).plot.data.y,~] = ...
                    GetCircleData(sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2),sim(iSim).sensor(iAgent,1).param.regionRadius);
                set(sim(iSim).sensor(iAgent,1).plot.fov,'Xdata',sim(iSim).sensor(iAgent,1).plot.data.x,'Ydata',sim(iSim).sensor(iAgent,1).plot.data.y);
                sim(iSim).sensor(iAgent,1).plot.hist.data.x(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.data.x';
                sim(iSim).sensor(iAgent,1).plot.hist.data.y(:,iClock+1) = sim(iSim).sensor(iAgent,1).plot.data.y';
                
                if sim(iSim).sensor(iAgent,1).plot.bDetect % when the sensor detects at least one of targets
                    set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.detect);
                else
                    set(sim(iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(iSim).sensor(iAgent,1).plot.clr.noDetect);
                end
            end
            %-----------------------------------
            
            
            %-----------------------------------
            % particle measurement and agent state sharing through communication
            [sim(iSim).comm(iAgent).beta,sim(iSim).comm(iAgent).bConnect,sim(iSim).planner(iAgent).agent,sim(iSim).comm(iAgent).z] = ...
                ShareInformation(sim(iSim).agent,sim(iSim).sensor,sim(iSim).planner(iAgent).agent,sim(iSim).filter(iAgent).id(1), sim(iSim).flagComm);
            sim(iSim).comm(iAgent).hist.beta(:,iClock+1) = sim(iSim).comm(iAgent).beta;
            sim(iSim).comm(iAgent).hist.bConnect(:,iClock+1) = sim(iSim).comm(iAgent).bConnect;
            sim(iSim).comm(iAgent).hist.Z(:,:,iClock+1) = sim(iSim).comm(iAgent).z';
            %-----------------------------------
            
            %-----------------------------------
            % Actual measurement and estimation: PF
            %
            % PF is locally performend, and measurement information is delivered
            % under the communication probability
            for iTarget = 1:sim(iSim).nTarget
                % particle state update
                sim(iSim).filter(iAgent,iTarget).pt = UpdateParticle(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).param,sim(iSim).clock.dt);
                
                % particle weight update
                sim(iSim).filter(iAgent,iTarget).w = UpdateParticleWeight(sim(iSim).comm(iAgent).z(:,iTarget),sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).agent,sim(iSim).sensor(iAgent).param);
                
                % compute actual entropy for comparison
                targetUpdatePdf = ComputePDFMixture(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
                sim(iSim).filter(iAgent,iTarget).Hbefore = ComputeEntropy(targetUpdatePdf,sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
                sim(iSim).filter(iAgent,iTarget).hist.Hbefore(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).Hbefore;
                
                
                % update plot
                % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
                % PLOTTING SPACE!
                if (sim(iSim).flagPlot) && (iAgent == 1)
                    figure(1+iAgent)
                    subplot(sim(iSim).filter(iAgent,iTarget).plot.location.col,...
                        sim(iSim).filter(iAgent,iTarget).plot.location.row,...
                        sim(iSim).filter(iAgent,iTarget).plot.location.num)
                    set(sim(iSim).filter(iAgent,iTarget).plot.targetPos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
                    set(sim(iSim).filter(iAgent,iTarget).plot.pt,'Xdata',sim(iSim).filter(iAgent,iTarget).pt(1,:),'Ydata',sim(iSim).filter(iAgent,iTarget).pt(2,:),...
                        'Cdata',sim(iSim).filter(iAgent,iTarget).w);
                    set(sim(iSim).filter(iAgent,iTarget).plot.targetId,'position',...
                        [sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
                    set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                        'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
                end
                
                
                % resample particle
                [sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w] = ResampleParticle(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).field);
                
                % particle filter info update/store
                sim(iSim).filter(iAgent,iTarget).xhat = (sim(iSim).filter(iAgent,iTarget).w*sim(iSim).filter(iAgent,iTarget).pt')';
                sim(iSim).filter(iAgent,iTarget).hist.pt(:,:,iClock+1) = sim(iSim).filter(iAgent,iTarget).pt;
                sim(iSim).filter(iAgent,iTarget).hist.w(:,:,iClock+1) = sim(iSim).filter(iAgent,iTarget).w;
                sim(iSim).filter(iAgent,iTarget).hist.xhat(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).xhat;
                
                
                % update planner initial info
                sim(iSim).planner(iAgent).PTset(iTarget).xhat = sim(iSim).filter(iAgent,iTarget).xhat;
                sim(iSim).planner(iAgent).PTset(iTarget).w = sim(iSim).filter(iAgent,iTarget).w;
                sim(iSim).planner(iAgent).PTset(iTarget).pt = sim(iSim).filter(iAgent,iTarget).pt;
                
                % store optimized infomation data
                sim(iSim).planner(iAgent).hist.actIdx(iClock+1) = sim(iSim).planner(iAgent).actIdx;
                sim(iSim).planner(iAgent).hist.input(:,iClock+1) = sim(iSim).planner(iAgent).input;
                
                % store entropy-based data for planner: only useful for MI-based planning
                if strcmp(sim(iSim).flagDM,'MI')
                    for iAgent = 1:sim(iSim).nAgent
                        sim(iSim).planner(iAgent).hist.I(:,iClock+1) = sim(iSim).planner(iAgent).I;
                        sim(iSim).planner(iAgent).hist.Hafter(:,iClock+1) = sim(iSim).planner(iAgent).Hafter';
                        sim(iSim).planner(iAgent).hist.Hbefore(:,iClock+1) = sim(iSim).planner(iAgent).Hbefore';
                        
                        sim(iSim).planner(iAgent).hist.IRef(:,iClock+1) = sim(iSim).planner(iAgent).IRef;
                        sim(iSim).planner(iAgent).hist.HafterRef(:,iClock+1) = sim(iSim).planner(iAgent).HafterRef';
                        sim(iSim).planner(iAgent).hist.HbeforeRef(:,iClock+1) = sim(iSim).planner(iAgent).HbeforeRef';
                    end
                end
                
                % compute actual entropy for comparison
                targetUpdatePdf = ComputePDFMixture(sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).filter(iAgent,iTarget).w,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);
                sim(iSim).filter(iAgent,iTarget).Hafter = ComputeEntropy(targetUpdatePdf,sim(iSim).filter(iAgent,iTarget).pt,sim(iSim).planner(iAgent).param,sim(iSim).flagPdfCompute);                
                sim(iSim).filter(iAgent,iTarget).hist.Hafter(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).Hafter;
                
                % compute actual entropy reduction for comparison
                sim(iSim).filter(iAgent,iTarget).I = sim(iSim).filter(iAgent,iTarget).Hbefore - sim(iSim).filter(iAgent,iTarget).Hafter;
                sim(iSim).filter(iAgent,iTarget).hist.I(:,iClock+1) = sim(iSim).filter(iAgent,iTarget).I;
                
            end
            %-----------------------------------
            

            %-----------------------------------
            % take decision making for agent input
            sim(iSim).agent(iAgent).vel = sim(iSim).planner(iAgent).input(1);
            %-----------------------------------

            
        end
        
        % clock update
        sim(iSim).clock.hist.time(:,iClock+1) = iClock*sim(iSim).clock.dt;
        
        % update figure
        if sim(iSim).flagPlot
            drawnow;
            figure(1)
            title(sprintf('t = %.1f [sec]', sim(iSim).clock.hist.time(:,iClock+1)))
        end

    end

    
    % display current simulation number
    fprintf('iSim = %d\n',iSim);
    
    
end

if sim(iSim).flagLog
    close all;
    save(['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'.mat']);
end

