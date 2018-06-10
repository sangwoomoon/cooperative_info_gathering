
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
sim.nAgent = 1;
sim.nTarget = 1;

sim.flagDM = 0; % 0: stationary agent | 1: optimization for agent motion control

if sim.flagDM == 1
    sim.flagDisp.before = 0;
    sim.flagDisp.after = 0;
else
    sim.flagDisp.before = 0;
    sim.flagDisp.after = 0;
end
%----------------------

%----------------------
% clock structure
clock.nt = 40;
clock.dt = 0.5;
clock.hist.time = 0;
%----------------------

%----------------------
% field structure
field.boundary = [-100 100 -100 100];
field.length = [field.boundary(2)-field.boundary(1) field.boundary(4)-field.boundary(3)];
field.buffer = 20; % for particle initalization
field.bufferZone = [field.boundary(1)+field.buffer field.boundary(2)-field.buffer...
    field.boundary(3)+field.buffer field.boundary(4)-field.buffer];
field.zoneLength = [field.bufferZone(2)-field.bufferZone(1) field.bufferZone(4)-field.bufferZone(3)];
%----------------------

%----------------------
% target structure
target.x = [30 40]'; % x_pos, y_pos
target.hist.x = target.x;
target.nState = length(target.x);
target.param.F = eye(length(target.x));
target.param.Q = zeros(target.nState); % certainly ideal
%----------------------

%----------------------
% agent structure
agent.s = [25 30 0 0]'; % x_pos, y_pos, heading, airspeed
agent.hist.s = agent.s;
%----------------------

%----------------------
% sensor structure
sensor.y = nan;
sensor.hist.y(:,1) = sensor.y;
sensor.param.regionRadius = 30; % sensing region radius
sensor.param.detectBeta = 0.85; % bernoulli detection parameter
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

planner.param.clock.dt = 5; % planning time-step horizon
planner.param.clock.nT = 3; % planning horizon

if sim.flagDM == 0
    planner.action = 0; % with respect to velocity
    planner.actionSet = zeros(planner.param.clock.nT,1);
    planner.actionSetNum = 1;
else
    planner.action = [0 -30*D2R 30*D2R]; % with respect to angular velocity
    planner.actionNum = length(planner.action);
    planner.actionSetNum = planner.actionNum^planner.param.clock.nT;
    for iAct = 1 : planner.param.clock.nT
        assignDen = planner.actionSetNum/planner.actionNum^iAct;
        repeatNum = planner.actionSetNum/assignDen;
        for jAct = 1 : repeatNum
            actionElem = planner.action(rem(jAct,planner.actionNum)+1);
            for kAct = 1 : assignDen
                planner.actionSet(iAct,assignDen*(jAct-1)+kAct) = actionElem;
            end
        end
    end
end

planner.nPt = PF.nPt;
planner.pt = PF.pt;
planner.w = PF.w;
planner.nState = PF.nState;

planner.input = nan(planner.param.clock.nT,1);
planner.actIdx = nan;
planner.hist.input = planner.input;
planner.hist.actIdx = planner.actIdx;

planner.I = nan;
planner.hist.I = planner.I;
planner.hist.Hbefore = nan(planner.param.clock.nT,1);
planner.hist.Hafter = nan(planner.param.clock.nT,1);

planner.param.pdf.dRefPt = 10;
[planner.param.pdf.refPt(:,:,1), planner.param.pdf.refPt(:,:,2)] = ...
    meshgrid(field.boundary(1):planner.param.pdf.dRefPt:field.boundary(2),field.boundary(3):planner.param.pdf.dRefPt:field.boundary(4));

planner.param.plot.row = planner.param.clock.nT; 
planner.param.plot.col = 2;

planner.param.F = target.param.F;
planner.param.Q = PF.param.Q;
planner.param.sensor.regionRadius = sensor.param.regionRadius;
planner.param.sensor.detectBeta = sensor.param.detectBeta;

%----------------------


%-----------------------------------
% Linear-Gaussian M.I. computation
%-----------------------------------

for iClock = 1:clock.nt    
    
    %-----------------------------------
    % PF-based Mutual information Computation and decision-making
    %-----------------------------------

    % compute future information with respect to action profiles
    for iActSet = 1 : planner.actionSetNum
        [planner.candidate.Hbefore(:,iActSet),planner.candidate.Hafter(:,iActSet),planner.candidate.I(iActSet)] = ...
            ComputeInformation(planner,agent,sensor,field,planner.param.clock,PF,sim,iActSet,iClock);
    end
    
    % direct decision making: maximize mutual information
    [~,planner.actIdx] = max(planner.candidate.I);
    
    % take optimized information data
    planner.input = planner.actionSet(:,planner.actIdx);
    planner.I = planner.candidate.I(planner.actIdx);
    planner.Hbefore = planner.candidate.Hbefore(:,planner.actIdx);
    planner.Hafter = planner.candidate.Hafter(:,planner.actIdx);
    
    
    
    %-----------------------------------
    % Actual Agent-Target Dynamics and Measurement
    %-----------------------------------
    
    % agent moving
    agent.s = UpdateAgentState(agent.s,planner.input(1),clock.dt);
    agent.hist.s(:,iClock+1) = agent.s;
    
    % target moving
    target.x = UpdateTargetState(target.x,target.param,clock.dt);
    target.hist.x(:,iClock+1) = target.x;
    
    % take measurement
    sensor.y = TakeMeasurement(target.x,agent.s,sensor.param);
    sensor.hist.y(:,iClock+1) = sensor.y;

    
    
    %-----------------------------------
    % Actual measurement and estimation: PF
    %-----------------------------------
    
    % particle state update
    PF.pt = UpdateParticle(PF.pt,PF.param,clock.dt);

    % particle measurement update
    PF.w = UpdateParticleWeight(sensor.y,PF.pt,agent.s,sensor.param);
            
    % resample particle
    [PF.pt,PF.w] = ResampleParticle(PF.pt,PF.w,field);
    
    % particle filter info update/store
    PF.xhat = (PF.w*PF.pt')';
    PF.hist.pt(:,:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = PF.xhat;
    
    % update planner initial info from PF results
    planner.x = PF.xhat;
    planner.w = PF.w;
    planner.pt = PF.pt;
    
    % store optimized infomation data
    planner.hist.actIdx(iClock+1) = planner.actIdx;
    planner.hist.input(:,iClock+1) = planner.input;
    planner.hist.I(:,iClock+1) = planner.I;
    planner.hist.Hafter(:,iClock+1) = planner.Hafter';
    planner.hist.Hbefore(:,iClock+1) = planner.Hbefore';
    
    % take decision making for agent input
    agent.vel = planner.input(1);
    
    % clock update
    clock.hist.time(:,iClock+1) = iClock*clock.dt;
    
    fprintf('iClock = %d\n',iClock);
    
end

%----------------------------
% Sim Result Plot
%----------------------------

figure(1)
plot(target.hist.x(1,:),target.hist.x(2,:),'r-','LineWidth',2); hold on;
plot(target.hist.x(1,1),target.hist.x(2,1),'ro','LineWidth',2); hold on;
plot(target.hist.x(1,end),target.hist.x(2,end),'rx','LineWidth',2); hold on;

plot(agent.hist.s(1,:),agent.hist.s(2,:),'b-','LineWidth',2); hold on;
plot(agent.hist.s(1,1),agent.hist.s(2,1),'bo','LineWidth',2); hold on;
plot(agent.hist.s(1,end),agent.hist.s(2,end),'bx','LineWidth',2); hold on;

plot(PF.hist.xhat(1,:),PF.hist.xhat(2,:),'m-','LineWidth',2); hold on;
plot(PF.hist.xhat(1,1),PF.hist.xhat(2,1),'mo','LineWidth',2); hold on;
plot(PF.hist.xhat(1,end),PF.hist.xhat(2,end),'mx','LineWidth',2); hold on;

xlabel('time [sec]'); ylabel('position [m]');

title('Target Tracking Trajectory and Estimates');
legend('true pos','true pos (start)','true pos (end)',...
    'agent pos','agent pos (start)','agent pos (end)',...
    'estimated pos','estimated pos (start)','estimated pos (end)');


figure(2)
plot(clock.hist.time,planner.hist.I,'b-','LineWidth',3);
xlabel('time [sec]'); ylabel('utility [sum of M.I.]');
title('Utility Profile');


figure(3)
[timePlanProfile,timeProfile] = meshgrid(1:planner.param.clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,planner.hist.Hbefore');
surface(timePlanProfile,timeProfile,planner.hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('particle-H[P(x_t|y_{k+1:t-1})]','particle-H[P(x_t|y_{k+1:t})]');
view(3);
title('Entropy Profile');

