
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
nSim = 1; % for Monte-Carlo approach


for iSim = 1:nSim
    
    %-------------------------------
    %   sim setting
    %-------------------------------
    
    %----------------------
    % simulation structure
    sim(iSim).nAgent = 2;
    sim(iSim).nTarget = 3;
    
    sim(iSim).flagScene = 0; % 0: stationary ground station | 1: moving ground station
    sim(iSim).flagInfoCom = 1; % 0: Ryan's approach | 1: Our approach(consider all measurements)
    sim(iSim).flagDM = 0; % 0: determined decisions | 1: DM enabled
    sim(iSim).flagPDF = 0; % 0: no PDF draw | 1: PDF draw
    
    if sim(iSim).flagPDF == 0
        sim(iSim).flagDisp.before = 0;
        sim(iSim).flagDisp.after = 0;
    else
        sim(iSim).flagDisp.before = 1;
        sim(iSim).flagDisp.after = 1;
    end
    
    sim(iSim).param.plot.dClock = 10; % interval of snapshot of particle evolution plotting
    %----------------------
    
    %----------------------
    % clock structure
    sim(iSim).clock.nt = 10;
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
    %----------------------
    
    %----------------------
    % target structure
    for iTarget = 1:sim(iSim).nTarget
        sim(iSim).target(iTarget).id = iTarget;
        sim(iSim).target(iTarget).x = ...
            [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1) sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)]'; % x_pos, y_pos
        sim(iSim).target(iTarget).hist.x = sim(iSim).target(iTarget).x;
        sim(iSim).target(iTarget).nState = length(sim(iSim).target(iTarget).x);
        sim(iSim).target(iTarget).param.F = eye(length(sim(iSim).target(iTarget).x));
        sim(iSim).target(iTarget).param.Q = zeros(sim(iSim).target(iTarget).nState); % certainly ideal
    end
    %----------------------
    
    %----------------------
    % agent structure
    radius = 50; % radius of fleet of agent
    dAngle = 360*D2R/sim(iSim).nAgent;
    
    for iAgent = 1:sim(iSim).nAgent
        sim(iSim).agent(iAgent).id = iAgent;
        sim(iSim).agent(iAgent).s = [radius*cos(iAgent*dAngle) radius*sin(iAgent*dAngle) iAgent*dAngle 5]'; % ring arrange
        sim(iSim).agent(iAgent).hist.s = sim(iSim).agent(iAgent).s;
    end
    %----------------------
    
    %----------------------
    % sensor structure
    for iSensor = 1:sim(iSim).nAgent
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).sensor(iSensor,iTarget).id = [iSensor,iTarget];
            sim(iSim).sensor(iSensor,iTarget).y = nan;
            sim(iSim).sensor(iSensor,iTarget).hist.y(:,1) = sim(iSim).sensor(iSensor,iTarget).y;
            sim(iSim).sensor(iSensor,iTarget).param.regionRadius = 50; % sensing region radius
            sim(iSim).sensor(iSensor,iTarget).param.detectBeta = 0.9; % bernoulli detection parameter
        end
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
                        [sim(iSim).field.bufferZone(1)+rand()*sim(iSim).field.zoneLength(1) sim(iSim).field.bufferZone(3)+rand()*sim(iSim).field.zoneLength(2)]';
                end
            else
                sim(iSim).PF(iAgent,iTarget).pt = sim(iSim).PF(1,iTarget).pt; % in order to make the same initial condition
            end
            sim(iSim).PF(iAgent,iTarget).hist.pt = sim(iSim).PF(iAgent,iTarget).pt;
            sim(iSim).PF(iAgent,iTarget).param.F = sim(iSim).target(iTarget).param.F; % assume target is stationary in PF
            sim(iSim).PF(iAgent,iTarget).param.Q = diag([20^2,20^2]);
            sim(iSim).PF(iAgent,iTarget).param.field = sim(iSim).field;
            sim(iSim).PF(iAgent,iTarget).nState = sim(iSim).target(iTarget).nState;
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
            GenerateOutcomeProfile([-12,0,12]*D2R,sim(iSim).planner(iPlanner).param.clock.nT);
        
        % measurement profile setting
        [sim(iSim).planner(iPlanner).meas,sim(iSim).planner(iPlanner).measNum,sim(iSim).planner(iPlanner).measSetNum,sim(iSim).planner(iPlanner).measSet] = ...
            GenerateOutcomeProfile([0 1],sim(iSim).planner(iPlanner).param.clock.nT);
        
        % communication profile setting
        [sim(iSim).planner(iPlanner).comm,sim(iSim).planner(iPlanner).commNum,sim(iSim).planner(iPlanner).commSetNum,sim(iSim).planner(iPlanner).commSet] = ...
            GenerateOutcomeProfile([0 1],sim(iSim).planner(iPlanner).param.clock.nT);
        
        
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
        
        sim(iSim).planner(iPlanner).param.pdf.dRefPt = 20;
        [sim(iSim).planner(iPlanner).param.pdf.refPt(:,:,1), sim(iSim).planner(iPlanner).param.pdf.refPt(:,:,2)] = ...
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
                ComputePDFMixture(sim(iSim).PF(iPlanner,iTarget).pt,sim(iSim).PF(iPlanner,iTarget).w,sim(iSim).planner(iPlanner).param,'Gaussian');
            sim(iSim).PF(iPlanner,iTarget).H = ComputeEntropy(targetUpdatePdf,sim(iSim).planner(iPlanner).param.pdf.dRefPt,'moon');
            sim(iSim).PF(iPlanner,iTarget).hist.H(:,1) = sim(iSim).PF(iPlanner,iTarget).H;
        end
    end
    %----------------------
    
    
    %-----------------------------------
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

                    if sim(iSim).flagInfoCom == 0
                        % Ryan's approach-based Mutual Information computation: Measurement sampling-based
                        [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction)] = ...
                            ComputeInformation(sim(iSim).planner(iAgent),agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,sim(iSim),iAction,iClock);
                    elseif sim(iSim).flagInfoCom == 1
                        % Mutual Information computation: Consider all future measurements
                        % consider communicaiton awareness
                        [sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction),sim(iSim).planner(iAgent).candidate.Hafter(:,iAction),sim(iSim).planner(iAgent).candidate.I(iAction)] = ...
                            ComputeInformationMeasConsider(sim(iSim).planner(iAgent),sim(iSim).agent,sim(iSim).field,sim(iSim).planner(iAgent).param.clock,sim(iSim).flagDisp,iAction,iClock,sim(iSim).agent(iAgent).id);
                    end
                    
                else % out of geofence
                    sim(iSim).planner(iAgent).candidate.Hbefore(:,iAction) = inf;
                    sim(iSim).planner(iAgent).candidate.Hafter(:,iAction) = inf;
                    sim(iSim).planner(iAgent).candidate.I(iAction) = inf;
                end
            end
            
            % decision making: maximize mutual information
            [~,sim(iSim).planner(iAgent).actIdx] = max(sim(iSim).planner(iAgent).candidate.I);
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
            sim(iSim).agent(iAgent).s = UpdateAgentState(sim(iSim).agent(iAgent).s,sim(iSim).planner(iAgent).input(1),sim(iSim).clock.dt);
            sim(iSim).agent(iAgent).hist.s(:,iClock+1) = sim(iSim).agent(iAgent).s;
        end
        
        % target moving
        for iTarget = 1:sim(iSim).nTarget
            sim(iSim).target(iTarget).x = UpdateTargetState(sim(iSim).target(iTarget).x,sim(iSim).target(iTarget).param,sim(iSim).clock.dt);
            sim(iSim).target(iTarget).hist.x(:,iClock+1) = sim(iSim).target(iTarget).x;
        end
        
        % take measurement
        for iSensor = 1:sim(iSim).nAgent
            for iTarget = 1:sim(iSim).nTarget
                sim(iSim).sensor(iSensor,iTarget).y = ...
                    TakeMeasurement(sim(iSim).target(iTarget).x,sim(iSim).agent(iSensor).s,sim(iSim).sensor(iSensor,iTarget).param);
                sim(iSim).sensor(iSensor,iTarget).hist.y(:,iClock+1) = sim(iSim).sensor(iSensor,iTarget).y;
            end
        end
        
        % particle measurement and agent state sharing through communication
        for iComm = 1:sim(iSim).nAgent
            [sim(iSim).comm(iComm).beta,sim(iSim).comm(iComm).bConnect,sim(iSim).planner(iComm).agent,sim(iSim).comm(iComm).z] = ...
                ShareInformation(sim(iSim).agent,sim(iSim).sensor,sim(iSim).planner(iComm).agent,sim(iSim).PF(iComm).id(1));
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
                targetUpdatePdf = ComputePDFMixture(sim(iSim).PF(iPF,iTarget).pt,sim(iSim).PF(iPF,iTarget).w,sim(iSim).planner(iPF).param,'Gaussian');
                sim(iSim).PF(iPF,iTarget).H = ComputeEntropy(targetUpdatePdf,sim(iSim).planner(iPF).param.pdf.dRefPt,'moon');
                sim(iSim).PF(iPF,iTarget).hist.H(:,iClock+1) = sim(iSim).PF(iPF,iTarget).H;
                
            end
        end
        
        % clock update
        sim(iSim).clock.hist.time(:,iClock+1) = iClock*sim(iSim).clock.dt;
        
        % display current clock when this script does not use Monte-Carlo
        if nSim == 1
            fprintf('iClock = %d\n',iClock);
        end
        
    end

    
    % display current simulation number
    fprintf('iSim = %d\n',iSim);
    
    
end

%%
%----------------------------
% Sim Result Plot
%----------------------------

% pick one of simulation set
rSim = ceil(rand(1)*nSim);

% aircraft trajectories and estimated target location
figure(1)
for iTarget = 1:sim(iSim).nTarget
    plot(sim(rSim).target(iTarget).hist.x(1,:),sim(rSim).target(iTarget).hist.x(2,:),'r-','LineWidth',2); hold on;
    plot(sim(rSim).target(iTarget).hist.x(1,1),sim(rSim).target(iTarget).hist.x(2,1),'ro','LineWidth',2); hold on;
    plot(sim(rSim).target(iTarget).hist.x(1,end),sim(rSim).target(iTarget).hist.x(2,end),'rx','LineWidth',2); hold on;
end

for iAgent = 1:sim(iSim).nAgent
    clr = rand(1,3);
    plot(sim(rSim).agent(iAgent).hist.s(1,:),sim(rSim).agent(iAgent).hist.s(2,:),'-','LineWidth',2,'color',clr); hold on;
    plot(sim(rSim).agent(iAgent).hist.s(1,1),sim(rSim).agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr); hold on;
    plot(sim(rSim).agent(iAgent).hist.s(1,end),sim(rSim).agent(iAgent).hist.s(2,end),'x','LineWidth',2,'color',clr); hold on;
end

% only for agent 1's info
for iTarget = 1:sim(iSim).nTarget
    clr = rand(1,3);
    plot(sim(rSim).PF(1,iTarget).hist.xhat(1,:),sim(rSim).PF(1,iTarget).hist.xhat(2,:),'--','LineWidth',2,'color',clr); hold on;
    plot(sim(rSim).PF(1,iTarget).hist.xhat(1,1),sim(rSim).PF(1,iTarget).hist.xhat(2,1),'o','LineWidth',2,'color',clr); hold on;
    plot(sim(rSim).PF(1,iTarget).hist.xhat(1,end),sim(rSim).PF(1,iTarget).hist.xhat(2,end),'x','LineWidth',2,'color',clr); hold on;
end


xlabel('time [sec]'); ylabel('position [m]');

title('Target Tracking Trajectory and Estimates');
% legend('true pos','true pos (start)','true pos (end)',...
%     'Agent 1 pos','Agent 1 pos (start)','Agent 1 (end)',...
%     'GS estimated pos','GS estimated pos (start)','GS estimated pos (end)',...
%     'Agent 2 pos','Agent 2 pos (start)','Agent 2 pos (end)',...
%     'Agent 2 estimated pos','Agent 2 estimated pos (start)','Agent 2 estimated pos (end)');
% 
axis equal; axis(sim(rSim).field.boundary);

%%
% snapshots and particles
figure(2)
if rem(sim(rSim).clock.nt,sim(rSim).param.plot.dClock) == 0
    nSnapshot = floor(sim(rSim).clock.nt/sim(rSim).param.plot.dClock)+1; % initial, during(dClock/each)-1, final
else
    nSnapshot = floor(sim(rSim).clock.nt/sim(rSim).param.plot.dClock)+2; % initial, during(dClock/each), final
end
nCol = 3;
nRow = ceil(nSnapshot/nCol);

clr = [1 0 0; 0 0 1; 1 1 0];
ptClr = [1 0 0; 0 0 1; 1 1 0];

for iSnapshot = 1:nSnapshot
    subplot(nRow,nCol,iSnapshot)
    if iSnapshot == 1
        SnapTime = 1;
    elseif iSnapshot == nSnapshot
        SnapTime = sim(rSim).clock.nt+1;
    else
        SnapTime = (iSnapshot-1)*sim(rSim).param.plot.dClock+1;
    end
    
    % target position
    for iTarget = 1:sim(iSim).nTarget % marked as green
        plot(sim(rSim).target(iTarget).hist.x(1,1:SnapTime),sim(rSim).target(iTarget).hist.x(2,1:SnapTime),'g-','LineWidth',2); hold on;
        plot(sim(rSim).target(iTarget).hist.x(1,1),sim(rSim).target(iTarget).hist.x(2,1),'go','LineWidth',2); hold on;
        plot(sim(rSim).target(iTarget).hist.x(1,SnapTime),sim(rSim).target(iTarget).hist.x(2,SnapTime),'gx','LineWidth',2); hold on;
    end
    
    % agent position
    for iAgent = 1:sim(iSim).nAgent
        plot(sim(rSim).agent(iAgent).hist.s(1,1:SnapTime),sim(rSim).agent(iAgent).hist.s(2,1:SnapTime),'-','LineWidth',2,'color',clr(iAgent,:)); hold on;
        plot(sim(rSim).agent(iAgent).hist.s(1,1),sim(rSim).agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr(iAgent,:)); hold on;
        plot(sim(rSim).agent(iAgent).hist.s(1,SnapTime),sim(rSim).agent(iAgent).hist.s(2,SnapTime),'x','LineWidth',2,'color',clr(iAgent,:)); hold on;
        
        % particle plotting
        plot(squeeze(sim(rSim).PF(iAgent).hist.pt(1,:,SnapTime)),squeeze(sim(rSim).PF(iAgent).hist.pt(2,:,SnapTime)),'.','LineWidth',2,'color',ptClr(iAgent,:)); hold on;
    end
    
    axis equal; axis(sim(rSim).field.boundary);
    xlabel(['k = ',num2str((SnapTime-1)*sim(rSim).clock.dt),' sec'],'fontsize',12);
end


% utility profile
figure(3)
for iAgent = 1:sim(iSim).nAgent
    plot(sim(rSim).clock.hist.time,sim(rSim).planner(iAgent).hist.I,'color',clr(iAgent,:),'LineWidth',3); hold on;
end
xlabel('time [sec]'); ylabel('\Sigma_{t=k+1}^{k+T} I(X_t;{^iZ_t}|{^iZ_{k+1:t}}), [nats]');
legend('Imperfect communication','Perfect communication');
% title('Utility Profile');


% utility profile: with respect to single M.I.
figure(4)
[timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(1).param.clock.nT,sim(rSim).clock.hist.time);
mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hbefore'-sim(rSim).planner(2).hist.Hafter');
surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hbefore'-sim(rSim).planner(1).hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('mutual information');
legend('particle-I(x_t;y_t)','particle-I(x_t;z_t)');
title('Mutual Information Profile');


% entropy profile
figure(5)
[timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(1).param.clock.nT,sim(rSim).clock.hist.time);
mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hbefore');
surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hbefore');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('H[P(x_t|y_{k+1:t-1})]','H[P(x_t|z_{k+1:t-1})]');
view(3);
title('Entropy of Prior Probability');


% entropy profile
figure(6)
[timePlanProfile,timeProfile] = meshgrid(1:sim(rSim).planner(2).param.clock.nT,sim(rSim).clock.hist.time);
mesh(timePlanProfile,timeProfile,sim(rSim).planner(2).hist.Hafter');
surface(timePlanProfile,timeProfile,sim(rSim).planner(1).hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('H[P(x_t|y_{k+1:t})]','H[P(x_t|z_{k+1:t})]');
view(3);
title('Entropy of Posterior Probability');


% prior/posterior entropy variation w.r.t one actual time steps
figure(7)
[timePlanProfile,timeProfile] = meshgrid(reshape(repmat(1:sim(rSim).planner(2).param.clock.nT,2,1),1,[]),sim(rSim).clock.hist.time);
for iClock = 1:sim(rSim).clock.nt+1
    entropyProfileGS(iClock,1:2:2*numel(sim(rSim).planner(1).Hbefore)-1) = sim(rSim).planner(1).hist.Hbefore(:,iClock);
    entropyProfileGS(iClock,2:2:2*numel(sim(rSim).planner(1).Hbefore)) = sim(rSim).planner(1).hist.Hafter(:,iClock);
    
    entropyProfilePerfectComm(iClock,1:2:2*numel(sim(rSim).planner(2).Hbefore)-1) = sim(rSim).planner(2).hist.Hbefore(:,iClock);
    entropyProfilePerfectComm(iClock,2:2:2*numel(sim(rSim).planner(2).Hbefore)) = sim(rSim).planner(2).hist.Hafter(:,iClock);
end
mesh(timePlanProfile,timeProfile,entropyProfileGS);
surface(timePlanProfile,timeProfile,entropyProfilePerfectComm);


% entropy profile from real PF estimation
figure(8)
for iAgent = 1:sim(iSim).nAgent
    plot(sim(rSim).clock.hist.time,sim(rSim).PF(iAgent).hist.H,'color',clr(iAgent,:),'LineWidth',3); hold on;
end
xlabel('time [sec]'); ylabel('entropy [nats]');
legend('Imperfect communication','Perfect communication');
% title('Utility Profile');


%%
%----------------------------
% Monte-Carlo based Information analysis
%----------------------------

if nSim > 1 % for multiple simulation results
    Imc = nan(2,nSim);
    for iSim = 1:nSim
        Imc(1,iSim) = sim(iSim).planner(1).Isum;
        Imc(2,iSim) = sim(iSim).planner(2).Isum;
        ImcRatio(iSim) = Imc(1,iSim)/Imc(2,iSim);
    end
    
    figure(101);
    for iAgent=1:sim(1).nAgent
    histogram(Imc(iAgent,:)',40,'FaceColor',clr(iAgent,:)); hold on;
    end
    legend('Imperfect communication','perfect communication');
    % title('Information Distribution: 100 Sims');
    xlabel('\Sigma_{k=1}^{20}\Sigma_{t=k+1}^{k+T} I(X_t;{^iZ_t}|{^iZ_{k+1:t-1}}), [nats]','Fontsize',12);
    ylabel('frequency','Fontsize',12);
    
    figure(102);
    histogram(ImcRatio,40);
    xlabel('Mutual information ratio','Fontsize',12);
    ylabel('frequency','Fontsize',12);
end