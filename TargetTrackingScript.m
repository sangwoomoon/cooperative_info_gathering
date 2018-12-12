
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
%
%   X(t+1) = X(t) + W                               : 2D-static Linear Gaussian
%   Y(t) = {0,1} with respect to Sensing Region     : 2D on the ground, circle region for target detection
%   s(t+1) = f(s(t),u(t))                           : u(t) = [0 -omega +omega]


close all;
clear;
% clc;
format compact;
hold on;

D2R = pi/180;
nSim = 1; % for Monte-Carlo approach or method comparison
targetID = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ';

for iSim = 1:nSim
    
    %-------------------------------
    %   sim setting
    %-------------------------------
    
    %----------------------
    % simulation structure
    sim(iSim).nAgent = 12;
    sim(iSim).nTarget = 3;
    
    sim(iSim).flagScene = 0; % 0: stationary ground station | 1: moving ground station
    sim(iSim).flagInfoCom = 1; % 0: Ryan's approach | 1: Our approach(consider all measurements)
    sim(iSim).flagDM = 'random'; % 0: 'random': random decision | 'MI': mutual information-based decision | 'mean': particle mean following
    sim(iSim).flagPDF = 0; % 0: no PDF draw | 1: PDF draw
    sim(iSim).flagComm = 0; % 0: perfect communication | 1: imperfect communication and communication awareness
    sim(iSim).flagPdfCompute = 'cylinder'; % 'uniform': uniformly discretized domain | 'cylinder': cylinder based computation w.r.t particle set
    sim(iSim).flagLog = 0;
    
    if ~sim(iSim).flagPDF
        sim(iSim).flagDisp.before = 0;
        sim(iSim).flagDisp.after = 0;
    else
        sim(iSim).flagDisp.before = 1;
        sim(iSim).flagDisp.after = 1;
    end
    
    sim(iSim).param.plot.dClock = 10; % interval of snapshot of particle evolution plotting
    
    % simulation plotting initialization:
    % Figure 1: agent/target moving
    sim(iSim).plot.fieldView = figure(1); hold on;
    % Figure 2+ (# of agents): estimation. particle evolution
    % ONLY AGENT 1 PLOTS ESTIMATION RESULT BECAUSE OF HUGE PLOTTING SPACE!!
    %for iAgent = 1:sim(iSim).nAgent
        iAgent = 1;
        sim(iSim).plot.particle(iAgent) = figure(1+iAgent); hold on;
    %end
    %----------------------
    
    %----------------------
    % clock structure
    sim(iSim).clock.nt = 200;
    sim(iSim).clock.dt = 1;
    sim(iSim).clock.hist.time = 0;
    %----------------------
    
    %----------------------
    % field structure
    sim(iSim).field.boundary = [-300 300 -300 300];
    sim(iSim).field.length = [sim(iSim).field.boundary(2)-sim(iSim).field.boundary(1) sim(iSim).field.boundary(4)-sim(iSim).field.boundary(3)];
    sim(iSim).field.buffer = 50; % for particle initalization
    sim(iSim).field.bufferZone = [sim(iSim).field.boundary(1)+sim(iSim).field.buffer sim(iSim).field.boundary(2)-sim(iSim).field.buffer...
        sim(iSim).field.boundary(3)+sim(iSim).field.buffer sim(iSim).field.boundary(4)-sim(iSim).field.buffer];
    sim(iSim).field.zoneLength = [sim(iSim).field.bufferZone(2)-sim(iSim).field.bufferZone(1) sim(iSim).field.bufferZone(4)-sim(iSim).field.bufferZone(3)];
    
    % field plotting setting
    figure(1)
    set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
        'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
    set(sim(iSim).plot.fieldView,'color','w')
    xlabel('East [m]'); ylabel('North [m]'); axis equal;
    %----------------------
    
    %----------------------
    % target structure
    for iTarget = 1:sim(iSim).nTarget
        
        % basic property setting
        sim(iSim).target(iTarget).id = iTarget;
        sim(iSim).target(iTarget).x = ...
            [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1) sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)]'; % x_pos, y_pos
        sim(iSim).target(iTarget).hist.x = sim(iSim).target(iTarget).x;
        sim(iSim).target(iTarget).nState = length(sim(iSim).target(iTarget).x);
        sim(iSim).target(iTarget).param.F = eye(length(sim(iSim).target(iTarget).x));
        sim(iSim).target(iTarget).param.Q = zeros(sim(iSim).target(iTarget).nState); % certainly ideal
        
        % plotting parameters setting: red, square
        sim(iSim).target(iTarget).plot.clr = [1 rand() 0];
        sim(iSim).target(iTarget).plot.marker = 'square';
        
        % position plot setting
        sim(iSim).target(iTarget).plot.pos = ...
            plot(sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2),...
                sim(iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(iSim).target(iTarget).plot.clr);
        sim(iSim).target(iTarget).plot.id = ...
            text(sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2),...
                targetID(iTarget));
        
        % trajectory plot setting
        sim(iSim).target(iTarget).plot.path = animatedline(sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2));
    end
    %----------------------
    
    %----------------------
    % agent structure
    
    % parameters fleet of agent for initial positioning
    nSquad = ceil(sim(iSim).nAgent/4);
    iSquad = 1;
    for iAgent = 1:sim(iSim).nAgent
        % basic state setting
        sim(iSim).agent(iAgent).id = iAgent;
        
        % deploy agents as 4 groups: corner
        switch iSquad
            case 1
                pos = [sim(iSim).field.bufferZone(2)-rand()*sim(iSim).field.zoneLength(1)/4 sim(iSim).field.bufferZone(4)-rand()*sim(iSim).field.zoneLength(2)/4, 5/8*2*pi];
            case 2
                pos = [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1)/4 sim(iSim).field.bufferZone(4)-rand()*sim(iSim).field.zoneLength(2)/4, 7/8*2*pi];
            case 3
                pos = [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1)/4 sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)/4, 1/8*2*pi];
            case 4
                pos = [sim(iSim).field.bufferZone(2)-rand()*sim(iSim).field.zoneLength(1)/4 sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)/4, 3/8*2*pi];
        end
        
        sim(iSim).agent(iAgent).s = [pos 10]';
        sim(iSim).agent(iAgent).hist.s = sim(iSim).agent(iAgent).s;
        
        % plotting parameter setting: blue, circle color
        sim(iSim).agent(iAgent).plot.clr = [0 rand() 1];
        sim(iSim).agent(iAgent).plot.marker = 'o';
        
        % agent position plotting
        sim(iSim).agent(iAgent).plot.pos = ...
            plot(sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2),...
                sim(iSim).agent(iAgent).plot.marker,'LineWidth',2,'color',sim(iSim).agent(iAgent).plot.clr);
        sim(iSim).agent(iAgent).plot.id = ...
            text(sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2),...
                num2str(iAgent));
            
        % agent trajectory plotting
        sim(iSim).agent(iAgent).plot.path = animatedline(sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2));
        if mod(iAgent,nSquad) == 0
            iSquad = iSquad + 1;
        end
    end
    %----------------------
    
    %----------------------
    % sensor structure
    for iSensor = 1:sim(iSim).nAgent
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).sensor(iSensor,iTarget).id = [iSensor,iTarget];
            sim(iSim).sensor(iSensor,iTarget).y = nan;
            sim(iSim).sensor(iSensor,iTarget).hist.y(:,1) = sim(iSim).sensor(iSensor,iTarget).y;
            sim(iSim).sensor(iSensor,iTarget).param.regionRadius = 40; % sensing region radius
            sim(iSim).sensor(iSensor,iTarget).param.detectBeta = 0.9; % bernoulli detection parameter
        end
        
        % plotting parameter setting
        sim(iSim).sensor(iSensor,1).plot.clr.detect = [0 1 0]; % green
        sim(iSim).sensor(iSensor,1).plot.clr.noDetect = [1 0 0]; % red
        sim(iSim).sensor(iSensor,1).plot.clr.opaqueValue = 0.1; % transparent value
        sim(iSim).sensor(iSensor,1).plot.bDetect = 0; % binary value for sening alarm
        sim(iSim).sensor(iSensor,1).plot.hist.bDetect(:,1) = sim(iSim).sensor(iSensor,1).plot.bDetect;
        
        % field of view plot
        [sim(iSim).sensor(iSensor,1).plot.data.x,sim(iSim).sensor(iSensor,1).plot.data.y,sim(iSim).sensor(iSensor,1).plot.fov] = ...
            GetCircleData(sim(iSim).agent(iSensor).s(1),sim(iSim).agent(iSensor).s(2),...
            sim(iSim).sensor(iSensor,iTarget).param.regionRadius,...
            sim(iSim).sensor(iSensor,1).plot.clr.noDetect,...
            sim(iSim).sensor(iSensor,1).plot.clr.opaqueValue); hold on;
        sim(iSim).sensor(iSensor,1).plot.hist.data.x(:,1) = sim(iSim).sensor(iSensor,1).plot.data.x';
        sim(iSim).sensor(iSensor,1).plot.hist.data.y(:,1) = sim(iSim).sensor(iSensor,1).plot.data.y';
    end
    %----------------------
    
    %----------------------
    % communicaiton structure
    for iComm = 1:sim(iSim).nAgent
        sim(iSim).comm(iComm).id = iComm;
        sim(iSim).comm(iComm).beta = nan(sim(iSim).nAgent,1);
        sim(iSim).comm(iComm).bConnect = nan(sim(iSim).nAgent,1);
        sim(iSim).comm(iComm).hist.beta(:,1) = sim(iSim).comm(iComm).beta; % for all agent and agent itself
        sim(iSim).comm(iComm).hist.bConnect(:,1) = sim(iSim).comm(iComm).bConnect; % for all agent and agent itself
    end
    %----------------------
    
    %----------------------
    % filter structure (Particle Filter)
    for iAgent = 1:sim(iSim).nAgent
        
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).PF(iAgent,iTarget).id = [iAgent, iTarget];
            sim(iSim).PF(iAgent,iTarget).nPt = 100;
            sim(iSim).PF(iAgent,iTarget).w = ones(1,sim(iSim).PF(iAgent,iTarget).nPt)./sim(iSim).PF(iAgent,iTarget).nPt;
            if iAgent == 1
                for iPt = 1 : sim(iSim).PF(iAgent,iTarget).nPt
                    sim(iSim).PF(iAgent,iTarget).pt(:,iPt) = ...
                        [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1) ...
                            sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)]';
                end
            else
                sim(iSim).PF(iAgent,iTarget).pt = sim(iSim).PF(1,iTarget).pt; % in order to make the same initial condition
            end
            sim(iSim).PF(iAgent,iTarget).hist.pt = sim(iSim).PF(iAgent,iTarget).pt;
            sim(iSim).PF(iAgent,iTarget).param.F = sim(iSim).target(iTarget).param.F; % assume target is stationary in PF
            sim(iSim).PF(iAgent,iTarget).param.Q = diag([18^2,18^2]);
            sim(iSim).PF(iAgent,iTarget).param.field = sim(iSim).field;
            sim(iSim).PF(iAgent,iTarget).nState = sim(iSim).target(iTarget).nState;
            
            % plotting parameter setting
            sim(iSim).PF(iAgent,iTarget).plot.location.col = 2;
            sim(iSim).PF(iAgent,iTarget).plot.location.row = ceil(sim(iSim).nTarget/sim(iSim).PF(iAgent,iTarget).plot.location.col);
            sim(iSim).PF(iAgent,iTarget).plot.location.num = iTarget;
            
            sim(iSim).PF(iAgent,iTarget).plot.clr = 'magenta';
            sim(iSim).PF(iAgent,iTarget).plot.marker = '.';
            sim(iSim).PF(iAgent,iTarget).plot.size = 2;
            
            % particle scatter plot setting
            % AGENT 1 ONLY VISUALIZES PARTICLE BECAUSE OF HUGE PLOTTING
            % SPACE!
            if iAgent == 1
                figure(1+iAgent)
                subplot(sim(iSim).PF(iAgent,iTarget).plot.location.col,...
                    sim(iSim).PF(iAgent,iTarget).plot.location.row,...
                    sim(iSim).PF(iAgent,iTarget).plot.location.num),
                set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                    'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
                set(sim(iSim).plot.particle(iAgent),'color','w')
                xlabel('East [m]'); ylabel('North [m]'); axis equal; hold on;
                
                % actual target plot setting for comparison
                sim(iSim).PF(iAgent,iTarget).plot.targetPos = ...
                    plot(sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2),...
                    sim(iSim).target(iTarget).plot.marker,'LineWidth',2,'color',sim(iSim).target(iTarget).plot.clr);
                sim(iSim).PF(iAgent,iTarget).plot.targetId = ...
                    text(sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2),...
                    targetID(iTarget));
                
                % particle plot
                sim(iSim).PF(iAgent,iTarget).plot.pt = ...
                    scatter(sim(iSim).PF(iAgent,iTarget).pt(1,:),sim(iSim).PF(iAgent,iTarget).pt(2,:),10,sim(iSim).PF(iAgent,iTarget).w,'filled');
                
            end
        end
        
    end
    %----------------------
    
    %----------------------
    % planner structure
    
    for iPlanner = 1:sim(iSim).nAgent
        
        sim(iSim).planner(iPlanner).id = iPlanner;
        
        sim(iSim).planner(iPlanner).param.clock.dt = 3; % planning time-step horizon
        sim(iSim).planner(iPlanner).param.clock.nT = 1; % planning horizon
%         sim(iSim).planner(iPlanner).param.sA = 3; % sampled action
        
        % action profile setting
        [sim(iSim).planner(iPlanner).action,sim(iSim).planner(iPlanner).actionNum,sim(iSim).planner(iPlanner).actionSetNum,sim(iSim).planner(iPlanner).actionSet] = ...
            GenerateOutcomeProfile([-20,0,20]*D2R,sim(iSim).planner(iPlanner).param.clock.nT);
        
        % measurement profile setting
        [sim(iSim).planner(iPlanner).meas,sim(iSim).planner(iPlanner).measNum,sim(iSim).planner(iPlanner).measSetNum,sim(iSim).planner(iPlanner).measSet] = ...
            GenerateOutcomeProfile([0 1],sim(iSim).planner(iPlanner).param.clock.nT);
        
        % communication profile setting
        if sim(iSim).flagComm
            [sim(iSim).planner(iPlanner).comm,sim(iSim).planner(iPlanner).commNum,sim(iSim).planner(iPlanner).commSetNum,sim(iSim).planner(iPlanner).commSet] = ...
                GenerateOutcomeProfile([0 1],sim(iSim).planner(iPlanner).param.clock.nT);
        else
            [sim(iSim).planner(iPlanner).comm,sim(iSim).planner(iPlanner).commNum,sim(iSim).planner(iPlanner).commSetNum,sim(iSim).planner(iPlanner).commSet] = ...
                GenerateOutcomeProfile(1,sim(iSim).planner(iPlanner).param.clock.nT);
        end
        
        
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).planner(iPlanner).PTset(iTarget).nPt = sim(iSim).PF(iPlanner,iTarget).nPt;
            sim(iSim).planner(iPlanner).PTset(iTarget).pt = sim(iSim).PF(iPlanner,iTarget).pt;
            sim(iSim).planner(iPlanner).PTset(iTarget).w = sim(iSim).PF(iPlanner,iTarget).w;
            sim(iSim).planner(iPlanner).PTset(iTarget).nState = sim(iSim).PF(iPlanner,iTarget).nState;
        end
        
        sim(iSim).planner(iPlanner).input = nan(sim(iSim).planner(iPlanner).param.clock.nT,1);
        sim(iSim).planner(iPlanner).actIdx = nan;
        sim(iSim).planner(iPlanner).hist.input = sim(iSim).planner(iPlanner).input;
        sim(iSim).planner(iPlanner).hist.actIdx = sim(iSim).planner(iPlanner).actIdx;
        
        sim(iSim).planner(iPlanner).I = nan;
        sim(iSim).planner(iPlanner).hist.I = sim(iSim).planner(iPlanner).I;
        sim(iSim).planner(iPlanner).hist.Hbefore = nan(sim(iSim).planner(iPlanner).param.clock.nT,1);
        sim(iSim).planner(iPlanner).hist.Hafter = nan(sim(iSim).planner(iPlanner).param.clock.nT,1);
        
        sim(iSim).planner(iPlanner).param.pdf.dRefPt = 10;
        [sim(iSim).planner(iPlanner).param.pdf.refPt(1,:,:), sim(iSim).planner(iPlanner).param.pdf.refPt(2,:,:)] = ...
            meshgrid(sim(iSim).field.boundary(1):sim(iSim).planner(iPlanner).param.pdf.dRefPt:sim(iSim).field.boundary(2),...
                     sim(iSim).field.boundary(3):sim(iSim).planner(iPlanner).param.pdf.dRefPt:sim(iSim).field.boundary(4));
        
        sim(iSim).planner(iPlanner).param.plot.row = sim(iSim).planner(iPlanner).param.clock.nT;
        sim(iSim).planner(iPlanner).param.plot.col = 2;
        
        sim(iSim).planner(iPlanner).param.F = sim(iSim).PF(iPlanner,iTarget).param.F;
        sim(iSim).planner(iPlanner).param.Q = sim(iSim).PF(iPlanner,iTarget).param.Q;
        sim(iSim).planner(iPlanner).param.sensor.regionRadius = sim(iSim).sensor(iPlanner).param.regionRadius;
        sim(iSim).planner(iPlanner).param.sensor.detectBeta = sim(iSim).sensor(iPlanner).param.detectBeta;
        
        sim(iSim).planner(iPlanner).param.field = sim(iSim).field;
        
        % for Monte-Carlo based analysis for information distribution
        sim(iSim).planner(iPlanner).Isum = 0;
        sim(iSim).planner(iPlanner).HbeforeSum = 0;
        sim(iSim).planner(iPlanner).HafterSum = 0;
        
        % agent state is used for communication awareness
        for iAgent = 1:sim(iSim).nAgent
            sim(iSim).planner(iPlanner).agent(iAgent).s = sim(iSim).agent(iAgent).s;
        end
        
        % FOR PF PART: entropy computation since it uses planner parameters
        % compute entropy at the initial time step
        for iTarget = 1:sim(iSim).nTarget
            targetUpdatePdf = ...
                ComputePDFMixture(sim(iSim).PF(iPlanner,iTarget).pt,sim(iSim).PF(iPlanner,iTarget).w,sim(iSim).planner(iPlanner).param,sim(iSim).flagPdfCompute);
            sim(iSim).PF(iPlanner,iTarget).H = ComputeEntropy(targetUpdatePdf,sim(iSim).PF(iPlanner,iTarget).pt,sim(iSim).planner(iPlanner).param,sim(iSim).flagPdfCompute);
            sim(iSim).PF(iPlanner,iTarget).hist.H(:,1) = sim(iSim).PF(iPlanner,iTarget).H;
        end
        
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
            
            for iAction = 1 : sim(iSim).planner(1).actionSetNum
                
                % check whether decision has feasibility in terms of geofence
                state = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).actionSet(iAction),sim(iSim).clock.dt);
                if (state(1) > sim(iSim).field.bufferZone(1) && state(1) < sim(iSim).field.bufferZone(2)) ...
                        && (state(2) > sim(iSim).field.bufferZone(3) && state(2) < sim(iSim).field.bufferZone(4)) % inside geofence

                    if ~sim(iSim).flagInfoCom
                        % Ryan's approach-based Mutual Information computation: Measurement sampling-based
                        [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction)] = ...
                            ComputeInformation(sim(iSim).planner(iAgent),agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,sim(iSim),iAction,iClock);
                    elseif sim(iSim).flagInfoCom
                        % Mutual Information computation: Consider all future measurements
                        % consider communicaiton awareness
                        [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction)] = ...
                            ComputeInformationMeasConsider(sim(iSim).planner(iAgent),sim(iSim).agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,...
                                sim(iSim).flagDisp,sim(iSim).flagComm,sim(iSim).flagPdfCompute,...
                                iAction,iClock,sim(iSim).agent(iAgent).id);
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
                sim(iSim).planner(iAgent).actIdx = ReturnToHome(sim(iSim).agent(iAgent).s);
            else
                [~,sim(iSim).planner(iAgent).actIdx] = max(sim(iSim).planner(iAgent).candidate.I);
            end
            sim(iSim).planner(iAgent).input = sim(iSim).planner(iAgent).actionSet(:,sim(iSim).planner(iAgent).actIdx);
            
            sim(iSim).planner(iAgent).I = sim(iSim).planner(iAgent).candidate.I(sim(iSim).planner(iAgent).actIdx);
            sim(iSim).planner(iAgent).Hbefore = sim(iSim).planner(iAgent).candidate.Hbefore(:,sim(iSim).planner(iAgent).actIdx);
            sim(iSim).planner(iAgent).Hafter = sim(iSim).planner(iAgent).candidate.Hafter(:,sim(iSim).planner(iAgent).actIdx);
            
            
            % add computed information to analyze Monte-Carlo based
            % approach
            sim(iSim).planner(iAgent).Isum = sim(iSim).planner(iAgent).Isum + sim(iSim).planner(iAgent).I;
            sim(iSim).planner(iAgent).HbeforeSum = sim(iSim).planner(iAgent).HbeforeSum + sim(iSim).planner(iAgent).Hbefore;
            sim(iSim).planner(iAgent).HafterSum = sim(iSim).planner(iAgent).HafterSum + sim(iSim).planner(iAgent).Hafter;
                
        end
            
        
        
        
        %-----------------------------------
        % Actual Agent-Target Dynamics and Measurement
        %-----------------------------------
        
        % agent moving
        for iAgent = 1:sim(iSim).nAgent
            % agent dynamics/store data
            sim(iSim).agent(iAgent).s = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).input(1),sim(iSim).clock.dt);
            sim(iSim).agent(iAgent).hist.s(:,iClock+1) = sim(iSim).agent(iAgent).s;
            
            % update plot 
            set(sim(iSim).agent(iAgent).plot.pos,'Xdata',sim(iSim).agent(iAgent).s(1),'Ydata',sim(iSim).agent(iAgent).s(2));
            set(sim(iSim).agent(iAgent).plot.id,'position',[sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2)]);
            addpoints(sim(iSim).agent(iAgent).plot.path,sim(iSim).agent(iAgent).s(1),sim(iSim).agent(iAgent).s(2));
            set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
        end
        
        % target moving
        for iTarget = 1:sim(iSim).nTarget
            % target dynamics/store data
            sim(iSim).target(iTarget).x = UpdateTargetState(sim(iSim).target(iTarget).x,sim(iSim).target(iTarget).param,sim(iSim).clock.dt);
            sim(iSim).target(iTarget).hist.x(:,iClock+1) = sim(iSim).target(iTarget).x;
            
            % update plot
            set(sim(iSim).target(iTarget).plot.pos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
            set(sim(iSim).target(iTarget).plot.id,'position',[sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);            
            addpoints(sim(iSim).target(iTarget).plot.path,sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2));
        end
        
        % take measurement
        for iSensor = 1:sim(iSim).nAgent
            
            sim(iSim).sensor(iSensor,1).plot.bDetect = 0;
            for iTarget = 1:sim(iSim).nTarget
                sim(iSim).sensor(iSensor,iTarget).y = ...
                    TakeMeasurement(sim(iSim).target(iTarget).x,sim(iSim).agent(iSensor).s,sim(iSim).sensor(iSensor,iTarget).param);
                sim(iSim).sensor(iSensor,iTarget).hist.y(:,iClock+1) = sim(iSim).sensor(iSensor,iTarget).y;
                
                if sim(iSim).sensor(iSensor,iTarget).y == 1
                    sim(iSim).sensor(iSensor,1).plot.bDetect = 1;
                end
            end
            sim(iSim).sensor(iSensor,1).plot.hist.bDetect(:,iClock+1) = sim(iSim).sensor(iSensor,1).plot.bDetect;
            
            % update plot
            [sim(iSim).sensor(iSensor,1).plot.data.x,sim(iSim).sensor(iSensor,1).plot.data.y,~] = ...
                GetCircleData(sim(iSim).agent(iSensor).s(1),sim(iSim).agent(iSensor).s(2),sim(iSim).sensor(iSensor,1).param.regionRadius);
            set(sim(iSim).sensor(iSensor,1).plot.fov,'Xdata',sim(iSim).sensor(iSensor,1).plot.data.x,'Ydata',sim(iSim).sensor(iSensor,1).plot.data.y);
            sim(iSim).sensor(iSensor,1).plot.hist.data.x(:,iClock+1) = sim(iSim).sensor(iSensor,1).plot.data.x';
            sim(iSim).sensor(iSensor,1).plot.hist.data.y(:,iClock+1) = sim(iSim).sensor(iSensor,1).plot.data.y';
            
            if sim(iSim).sensor(iSensor,1).plot.bDetect % when the sensor detects at least one of targets
                set(sim(iSim).sensor(iSensor,1).plot.fov,'FaceColor',sim(iSim).sensor(iSensor,1).plot.clr.detect);
            else
                set(sim(iSim).sensor(iSensor,1).plot.fov,'FaceColor',sim(iSim).sensor(iSensor,1).plot.clr.noDetect);
            end
        end
        
        % particle measurement and agent state sharing through communication
        for iComm = 1:sim(iSim).nAgent
            [sim(iSim).comm(iComm).beta,sim(iSim).comm(iComm).bConnect,sim(iSim).planner(iComm).agent,sim(iSim).comm(iComm).z] = ...
                ShareInformation(sim(iSim).agent,sim(iSim).sensor,sim(iSim).planner(iComm).agent,sim(iSim).PF(iComm).id(1), sim(iSim).flagComm);
            sim(iSim).comm(iComm).hist.beta(:,iClock+1) = sim(iSim).comm(iComm).beta;
            sim(iSim).comm(iComm).hist.bConnect(:,iClock+1) = sim(iSim).comm(iComm).bConnect;
            sim(iSim).comm(iComm).hist.Z(:,:,iClock+1) = sim(iSim).comm(iComm).z';
        end
        
        %-----------------------------------
        % Actual measurement and estimation: PF
        %-----------------------------------
        
        % PF is locally performend, and measurement information is delivered
        % under the communication probability
        for iPF = 1:sim(iSim).nAgent
            
            for iTarget = 1:sim(iSim).nTarget
                % particle state update
                sim(iSim).PF(iPF,iTarget).pt = UpdateParticle(sim(iSim).PF(iPF,iTarget).pt,sim(iSim).PF(iPF,iTarget).param,sim(iSim).clock.dt);
                
                % particle weight update
                sim(iSim).PF(iPF,iTarget).w = UpdateParticleWeight(sim(iSim).comm(iPF).z(:,iTarget),sim(iSim).PF(iPF,iTarget).pt,sim(iSim).planner(iPF).agent,sim(iSim).sensor(iPF).param);
                
                % update plot
                % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
                % PLOTTING SPACE!
                if iPF == 1
                    figure(1+iPF)
                    subplot(sim(iSim).PF(iPF,iTarget).plot.location.col,...
                        sim(iSim).PF(iPF,iTarget).plot.location.row,...
                        sim(iSim).PF(iPF,iTarget).plot.location.num)
                    set(sim(iSim).PF(iPF,iTarget).plot.targetPos,'Xdata',sim(iSim).target(iTarget).x(1),'Ydata',sim(iSim).target(iTarget).x(2));
                    set(sim(iSim).PF(iPF,iTarget).plot.pt,'Xdata',sim(iSim).PF(iPF,iTarget).pt(1,:),'Ydata',sim(iSim).PF(iPF,iTarget).pt(2,:),...
                        'Cdata',sim(iSim).PF(iPF,iTarget).w);
                    set(sim(iSim).PF(iPF,iTarget).plot.targetId,'position',...
                        [sim(iSim).target(iTarget).x(1),sim(iSim).target(iTarget).x(2)]);
                    set(gca,'xlim',[sim(iSim).field.boundary(1),sim(iSim).field.boundary(2)],...
                        'ylim',[sim(iSim).field.boundary(3),sim(iSim).field.boundary(4)])
                end
                
                
                % resample particle
                [sim(iSim).PF(iPF,iTarget).pt,sim(iSim).PF(iPF,iTarget).w] = ResampleParticle(sim(iSim).PF(iPF,iTarget).pt,sim(iSim).PF(iPF,iTarget).w,sim(iSim).field);
                
                % particle filter info update/store
                sim(iSim).PF(iPF,iTarget).xhat = (sim(iSim).PF(iPF,iTarget).w*sim(iSim).PF(iPF,iTarget).pt')';
                sim(iSim).PF(iPF,iTarget).hist.pt(:,:,iClock+1) = sim(iSim).PF(iPF,iTarget).pt;
                sim(iSim).PF(iPF,iTarget).hist.w(:,:,iClock+1) = sim(iSim).PF(iPF,iTarget).w;
                sim(iSim).PF(iPF,iTarget).hist.xhat(:,iClock+1) = sim(iSim).PF(iPF,iTarget).xhat;
                                
                % update planner initial info
                sim(iSim).planner(iPF).x = sim(iSim).PF(iPF,iTarget).xhat;
                sim(iSim).planner(iPF).w = sim(iSim).PF(iPF,iTarget).w;
                sim(iSim).planner(iPF).pt = sim(iSim).PF(iPF,iTarget).pt;
                
                % store optimized infomation data
                for iPlanner = 1:sim(iSim).nAgent
                    sim(iSim).planner(iPlanner).hist.actIdx(iClock+1) = sim(iSim).planner(iPlanner).actIdx;
                    sim(iSim).planner(iPlanner).hist.input(:,iClock+1) = sim(iSim).planner(iPlanner).input;
                    sim(iSim).planner(iPlanner).hist.I(:,iClock+1) = sim(iSim).planner(iPlanner).I;
                    sim(iSim).planner(iPlanner).hist.Hafter(:,iClock+1) = sim(iSim).planner(iPlanner).Hafter';
                    sim(iSim).planner(iPlanner).hist.Hbefore(:,iClock+1) = sim(iSim).planner(iPlanner).Hbefore';
                end
                
                % take decision making for agent input
                for iAgent = 1:sim(iSim).nAgent
                    sim(iSim).agent(iAgent).vel = sim(iSim).planner(iAgent).input(1);
                end
                
                % compute entropy for comparison
                targetUpdatePdf = ComputePDFMixture(sim(iSim).PF(iPF,iTarget).pt,sim(iSim).PF(iPF,iTarget).w,sim(iSim).planner(iPF).param,sim(iSim).flagPdfCompute);
                sim(iSim).PF(iPF,iTarget).H = ComputeEntropy(targetUpdatePdf,sim(iSim).PF(iPF,iTarget).pt,sim(iSim).planner(iPF).param,sim(iSim).flagPdfCompute);
                sim(iSim).PF(iPF,iTarget).hist.H(:,iClock+1) = sim(iSim).PF(iPF,iTarget).H;
                
            end
        end
        
        % clock update
        sim(iSim).clock.hist.time(:,iClock+1) = iClock*sim(iSim).clock.dt;
        
        % display current clock when this script does not use Monte-Carlo
        if nSim == 1
            fprintf('iClock = %d\n',iClock);
        end
        
        drawnow;
        
    end

    
    % display current simulation number
    fprintf('iSim = %d\n',iSim);
    
    
end

if sim(iSim).flagLog
    close all;
    save(['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'.mat']);
end

