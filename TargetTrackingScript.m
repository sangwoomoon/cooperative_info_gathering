
% Target Localization Problem Script
% Particle Method based Mutual Information
%
% X(t+1) ~ P_ta(X(t+1)|X(t))
% Y(t)   ~ P_se(Y(t)|X(t);s(t))
% 
% - coded by Sangwoo Moon
% - Created: 4/3/2018
% - 1st revision: 4/18/2018
%   X(t+1) = X(t) + W                               : 2D-static Linear Gaussian
%   Y(t) = {0,1} with respect to Sensing Region     : 2D on the ground, circle region for target detection
%   s(t+1) = f(s(t),u(t))                           : u(t) = [0 -omega +omega]


close all;
clear;
% clc;
format compact;
hold on;

D2R = pi/180;

%-------------------------------
%   sim setting
%-------------------------------

%----------------------
% simulation structure
sim.nAgent = 3;
sim.nTarget = 1;

sim.flagDM = 1; % 0: stationary agent | 1: optimization for agent motion control

if sim.flagDM == 1
    sim.flagDisp.before = 0;
    sim.flagDisp.after = 0;
else
    sim.flagDisp.before = 1;
    sim.flagDisp.after = 1;
end

sim.param.plot.dClock = 15; % interval of snapshot
%----------------------

%----------------------
% clock structure
clock.nt = 50;
clock.dt = 0.3;
clock.hist.time = 0;
%----------------------

%----------------------
% field structure
field.boundary = [-100 100 -100 100];
field.length = [field.boundary(2)-field.boundary(1) field.boundary(4)-field.boundary(3)];
field.buffer = 10; % for particle initalization
field.bufferZone = [field.boundary(1)+field.buffer field.boundary(2)-field.buffer...
    field.boundary(3)+field.buffer field.boundary(4)-field.buffer];
field.zoneLength = [field.bufferZone(2)-field.bufferZone(1) field.bufferZone(4)-field.bufferZone(3)];
%----------------------

%----------------------
% target structure
target.x = [20 10]'; % x_pos, y_pos
target.hist.x = target.x;
target.nState = length(target.x);
target.param.F = eye(length(target.x));
target.param.Q = zeros(target.nState); % certainly ideal
%----------------------

%----------------------
% agent structure

% specific setting
if sim.nAgent == 1
    agent(1).s = [-20 -20 rand()*2*pi 5]';
    agent(1).hist.s = agent(1).s;
else
    agent(1).s = [-30 -30 rand()*2*pi 5]';
    agent(1).hist.s = agent(1).s;
    agent(2).s = [-30 30 rand()*2*pi 5]';
    agent(2).hist.s = agent(2).s; 
    agent(3).s = [30 -30 rand()*2*pi 5]';
    agent(3).hist.s = agent(3).s;
end

% for iAgent = 1:sim.nAgent
%     agent(iAgent).s = [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2) rand()*2*pi 10]'; % x_pos, y_pos, heading, airspeed
%     agent(iAgent).hist.s = agent(iAgent).s;
% end
%----------------------

%----------------------
% sensor structure
for iSensor = 1:sim.nAgent
    sensor(iSensor).y = nan;
    sensor(iSensor).hist.y(:,1) = sensor(iSensor).y;
    sensor(iSensor).param.regionRadius = 10; % sensing region radius
    sensor(iSensor).param.detectBeta = 0.9; % bernoulli detection parameter
end
%----------------------

%----------------------
% filter structure (Particle Filter)
PF.nPt = 50;
PF.w = ones(1,PF.nPt)./PF.nPt;
for iPt = 1 : PF.nPt
    PF.pt(:,iPt) = [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2)]';
end
PF.hist.pt = PF.pt;
PF.param.F = target.param.F; % assume target is stationary in PF
PF.param.Q = diag([8^2,5^2]);
PF.nState = target.nState;
%----------------------

%----------------------
% planner structure

for iPlanner = 1:sim.nAgent
        
    planner(iPlanner).param.clock.dt = 3; % planning time-step horizon
    planner(iPlanner).param.clock.nT = 3; % planning horizon
    
    if sim.flagDM == 0
        planner(iPlanner).param.sA = 1; % sampled action
        planner(iPlanner).action = 0; % with respect to velocity
        planner(iPlanner).actionSet = zeros(planner(iPlanner).param.clock.nT,1);
        planner(iPlanner).actionSetNum = 1;
    else
        planner(iPlanner).param.sA = 20; % sampled action
        planner(iPlanner).action = [0 -60*D2R 60*D2R]; % with respect to angular velocity
        
        planner(iPlanner).actionNum = length(planner(iPlanner).action);
        planner(iPlanner).actionSetNum = planner(iPlanner).actionNum^(planner(iPlanner).param.clock.nT);
        
        for iAct = 1 : planner(iPlanner).param.clock.nT
            assignDen = planner(iPlanner).actionSetNum/planner(iPlanner).actionNum^iAct;
            repeatNum = planner(iPlanner).actionSetNum/assignDen;
            for jAct = 1 : repeatNum
                actionElem = planner(iPlanner).action(rem(jAct,planner(iPlanner).actionNum)+1);
                for kAct = 1 : assignDen
                    planner(iPlanner).actionSet(iAct,assignDen*(jAct-1)+kAct) = actionElem;
                end
            end
        end
        
    end
    
    planner(iPlanner).nPt = PF.nPt;
    planner(iPlanner).pt = PF.pt;
    planner(iPlanner).w = PF.w;
    planner(iPlanner).nState = PF.nState;
    
    planner(iPlanner).input = nan(planner(iPlanner).param.clock.nT,1);
    planner(iPlanner).actIdx = nan;
    planner(iPlanner).hist.input = planner(iPlanner).input;
    planner(iPlanner).hist.actIdx = planner(iPlanner).actIdx;
    
    planner(iPlanner).I = nan;
    planner(iPlanner).hist.I = planner(iPlanner).I;
    planner(iPlanner).hist.Hbefore = nan(planner(iPlanner).param.clock.nT,1);
    planner(iPlanner).hist.Hafter = nan(planner(iPlanner).param.clock.nT,1);
    
    planner(iPlanner).param.pdf.dRefPt = 20;
    [planner(iPlanner).param.pdf.refPt(:,:,1), planner(iPlanner).param.pdf.refPt(:,:,2)] = ...
        meshgrid(field.boundary(1):planner(iPlanner).param.pdf.dRefPt:field.boundary(2),field.boundary(3):planner(iPlanner).param.pdf.dRefPt:field.boundary(4));
    
    planner(iPlanner).param.plot.row = planner(iPlanner).param.clock.nT;
    planner(iPlanner).param.plot.col = 2;
    
    planner(iPlanner).param.F = target.param.F;
    planner(iPlanner).param.Q = PF.param.Q;
    planner(iPlanner).param.sensor.regionRadius = sensor(iPlanner).param.regionRadius;
    planner(iPlanner).param.sensor.detectBeta = sensor(iPlanner).param.detectBeta;
    
end
%----------------------


%-----------------------------------
% Linear-Gaussian M.I. computation
%-----------------------------------

for iClock = 1:clock.nt    
    
    %-----------------------------------
    % PF-based Mutual information Computation and decision-making
    %-----------------------------------

    % compute future information with respect to action profiles
    % 1. sample among action profile
    % 2. compute entropy/mutual information with respect to sampled action
    % profile
    %
    % Here assume agent 1 only operates the planner as if it is under
    % centralized scheme of which results are destributed to all agents
    sActNumSet = nan(sim.nAgent,planner(1).param.sA);
    for iSample = 1 : planner(1).param.sA
        
        % sample among action profile: depends on whether the simulation
        % uses optimization
        if sim.flagDM == 0
            sActNum = ones(sim.nAgent,1);
        else
            bRealSample = 1;
            bRealInput = ones(1,sim.nAgent);
            while (bRealSample)
                sActNum = 1+floor(rand(sim.nAgent,1)*planner(1).actionSetNum);
                for iAgent = 1:sim.nAgent
                    state = UpdateAgentState(agent(iAgent).s,planner(iAgent).actionSet(1,sActNum(iAgent,1)),clock.dt);
                    if (state(1) <= field.bufferZone(1) || state(1) >= field.bufferZone(2)) || (state(2) <= field.bufferZone(3) || state(2) >= field.bufferZone(4)) % out of geofence
                        bRealInput(iAgent) = 0;
                    end
                end
                
                if sum(bRealInput) == sim.nAgent
                    bRealSample = 0;
                end
            end
        end
        sActNumSet(:,iSample) = sActNum;
        
        [planner(1).candidate.Hbefore(:,iSample),planner(1).candidate.Hafter(:,iSample),planner(1).candidate.I(iSample)] = ...
            ComputeInformation(planner(1),agent,field,planner(1).param.clock,sim,sActNum,iClock);
    end
    
    % direct decision making: maximize mutual information
    [~,sActOptIdx] = max(planner(1).candidate.I);
    
    % distribute results from Agent 1's planner to all agents' planner: 
    % STRUCTURE NOTATION SHOULD BE MODIFIED FOR NOT TO MAKE CONFUSION!
    for iAgent = 1:sim.nAgent
        % take optimized information data
        planner(iAgent).actIdx = sActNumSet(iAgent,sActOptIdx);
        planner(iAgent).input = planner(iAgent).actionSet(:,planner(iAgent).actIdx);
        
        planner(iAgent).I = planner(1).candidate.I(sActOptIdx);
        planner(iAgent).Hbefore = planner(1).candidate.Hbefore(:,sActOptIdx);
        planner(iAgent).Hafter = planner(1).candidate.Hafter(:,sActOptIdx);
    end
    
    
    
    %-----------------------------------
    % Actual Agent-Target Dynamics and Measurement
    %-----------------------------------
    
    % agent moving
    for iAgent = 1:sim.nAgent
        agent(iAgent).s = UpdateAgentState(agent(iAgent).s,planner(iAgent).input(1),clock.dt);
        agent(iAgent).hist.s(:,iClock+1) = agent(iAgent).s;
    end
    
    % target moving
    target.x = UpdateTargetState(target.x,target.param,clock.dt);
    target.hist.x(:,iClock+1) = target.x;
    
    % take measurement
    for iSensor = 1:sim.nAgent
        sensor(iSensor).y = TakeMeasurement(target.x,agent(iSensor).s,sensor(iSensor).param);
        sensor(iSensor).hist.y(:,iClock+1) = sensor(iSensor).y;
    end
    
    
    %-----------------------------------
    % Actual measurement and estimation: PF
    %-----------------------------------
    
    % particle state update
    PF.pt = UpdateParticle(PF.pt,PF.param,clock.dt);

    % particle measurement update
    % ad-hoc: SHOULD BE MODIFIED!
    yAug = nan(1,sim.nAgent);
    for iSensor = 1:sim.nAgent
        yAug(1,iSensor) = sensor(iSensor).y;
        sAug(iSensor).s = agent(iSensor).s;
    end
    PF.w = UpdateParticleWeight(yAug,PF.pt,sAug,sensor(1).param);
            
    % resample particle
    [PF.pt,PF.w] = ResampleParticle(PF.pt,PF.w,field);
    
    % particle filter info update/store
    PF.xhat = (PF.w*PF.pt')';
    PF.hist.pt(:,:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = PF.xhat;
    
    % update planner initial info from PF results to all agents
    planner(1).x = PF.xhat;
    planner(1).w = PF.w;
    planner(1).pt = PF.pt;
    
    % store optimized infomation data
    for iPlanner = 1:sim.nAgent
        planner(iPlanner).hist.actIdx(iClock+1) = planner(iPlanner).actIdx;
        planner(iPlanner).hist.input(:,iClock+1) = planner(iPlanner).input;
        planner(iPlanner).hist.I(:,iClock+1) = planner(iPlanner).I;
        planner(iPlanner).hist.Hafter(:,iClock+1) = planner(iPlanner).Hafter';
        planner(iPlanner).hist.Hbefore(:,iClock+1) = planner(iPlanner).Hbefore';
    end
    
    % take decision making for agent input
    for iAgent = 1:sim.nAgent
        agent(iAgent).vel = planner(iAgent).input(1);
    end
    
    % clock update
    clock.hist.time(:,iClock+1) = iClock*clock.dt;
    
    fprintf('iClock = %d\n',iClock);
    
end

%----------------------------
% Sim Result Plot
%----------------------------

% aircraft trajectories and estimated target location
figure(1)
plot(target.hist.x(1,:),target.hist.x(2,:),'r-','LineWidth',2); hold on;
plot(target.hist.x(1,1),target.hist.x(2,1),'ro','LineWidth',2); hold on;
plot(target.hist.x(1,end),target.hist.x(2,end),'rx','LineWidth',2); hold on;

clr = nan(sim.nAgent,3);
for iAgent = 1:sim.nAgent
    clr(iAgent,:) = rand(1,3);
    plot(agent(iAgent).hist.s(1,:),agent(iAgent).hist.s(2,:),'-','LineWidth',2,'color',clr(iAgent,:)); hold on;
    plot(agent(iAgent).hist.s(1,1),agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr(iAgent,:)); hold on;
    plot(agent(iAgent).hist.s(1,end),agent(iAgent).hist.s(2,end),'x','LineWidth',2,'color',clr(iAgent,:)); hold on;
end

plot(PF.hist.xhat(1,:),PF.hist.xhat(2,:),'m-','LineWidth',2); hold on;
plot(PF.hist.xhat(1,1),PF.hist.xhat(2,1),'mo','LineWidth',2); hold on;
plot(PF.hist.xhat(1,end),PF.hist.xhat(2,end),'mx','LineWidth',2); hold on;

xlabel('time [sec]'); ylabel('position [m]');

title('Target Tracking Trajectory and Estimates');
legend('true pos','true pos (start)','true pos (end)',...
    'agent 1 pos','agent 1 pos (start)','agent 1 pos (end)',...
    'estimated pos','estimated pos (start)','estimated pos (end)');

axis equal; axis(field.boundary); 

% snapshots and particles
figure(2)

nSnapshot = floor(clock.nt/sim.param.plot.dClock);
nCol = 3;
nRow = floor(nSnapshot/nCol);

clr = [0 1 0; 0 0 1; 0 1 1];

for iSnapshot = 1:nSnapshot
    subplot(nRow,nCol,iSnapshot)
    if iSnapshot == 1
        SnapTime = 1;
    else
        SnapTime = (iSnapshot-1)*sim.param.plot.dClock;
    end
    
    plot(target.hist.x(1,1:SnapTime),target.hist.x(2,1:SnapTime),'r-','LineWidth',2); hold on;
    plot(target.hist.x(1,1),target.hist.x(2,1),'ro','LineWidth',2); hold on;
    plot(target.hist.x(1,SnapTime),target.hist.x(2,SnapTime),'rx','LineWidth',2); hold on;

    for iAgent = 1:sim.nAgent
        
        plot(agent(iAgent).hist.s(1,1:SnapTime),agent(iAgent).hist.s(2,1:SnapTime),'-','LineWidth',2,'color',clr(iAgent,:)); hold on;
        plot(agent(iAgent).hist.s(1,1),agent(iAgent).hist.s(2,1),'o','LineWidth',2,'color',clr(iAgent,:)); hold on;
        plot(agent(iAgent).hist.s(1,SnapTime),agent(iAgent).hist.s(2,SnapTime),'x','LineWidth',2,'color',clr(iAgent,:)); hold on;
    end
    
    plot(squeeze(PF.hist.pt(1,:,SnapTime)),squeeze(PF.hist.pt(2,:,SnapTime)),'m.','LineWidth',2); hold on;
    axis equal; axis(field.boundary); 
end


% utility profile
figure(3)
plot(clock.hist.time,planner(1).hist.I,'b-','LineWidth',3);
xlabel('time [sec]'); ylabel('utility [sum of M.I.]');
title('Utility Profile');


% entropy profile
figure(4)
[timePlanProfile,timeProfile] = meshgrid(1:planner(1).param.clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,planner(1).hist.Hbefore');
surface(timePlanProfile,timeProfile,planner(1).hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('particle-H[P(x_t|y_{k+1:t-1})]','particle-H[P(x_t|y_{k+1:t})]');
view(3);
title('Entropy Profile');

