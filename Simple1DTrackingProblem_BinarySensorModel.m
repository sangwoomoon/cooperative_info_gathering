
% Single Agent - Single Target Localization Problem
%
% X(t+1) = X(t) + W (linear Gaussian)
% Y(t) = {0,1} with respect to Sensing Region: 1D-based
% Particle filter based Mutual Information
% 
% - Quick and Dirty Ver.
% - coded by Sangwoo Moon
% - Created: 4/3/2018


close all;
clear;
% clc;
format compact;
hold on;

%-------------------------------
%   sim setting
%-------------------------------

%----------------------
% simulation structure
sim.nAgent = 1;
sim.nTarget = 1;
sim.flagDisp.before = 1;
sim.flagDisp.after = 1;
%----------------------

%----------------------
% clock structure
clock.nt = 10;
clock.dt = 1;
clock.nT = 5; % planning horizon
clock.hist.time = 0;
%----------------------

%----------------------
% field structure
field.boundary = [-100 100];
field.length = field.boundary(2) - field.boundary(1);
field.buffer = 20; % for particle initalization
field.bufferZone = [field.boundary(1)+field.buffer field.boundary(2)-field.buffer];
field.zoneLength = field.bufferZone(2) - field.bufferZone(1);
%----------------------

%----------------------
% target structure
target.pos = 0;
target.vel = 0;
target.hist.pos = target.pos;
%----------------------

%----------------------
% agent structure
agent.pos = 8;
agent.vel = 0;
agent.hist.pos = agent.pos;
agent.hist.vel = agent.vel;
%----------------------

%----------------------
% sensor structure
sensor.regionRadius = 10; % sensing region radius
sensor.DetectBeta = 0.85; % bernoulli detection parameter
sensor.y = nan;
sensor.hist.y(:,1) = sensor.y;
%----------------------

%----------------------
% filter structure (Particle Filter)
PF.xhat = 40;
PF.hist.xhat = PF.xhat;

PF.F = 1;
PF.Q = 7^2;

PF.nPt = 500;
PF.w = ones(1,PF.nPt)./PF.nPt;
for iPt = 1 : PF.nPt
    PF.pt(iPt,1) = field.bufferZone(1)+rand()*field.zoneLength;
end
PF.hist.pt = PF.pt;
%----------------------

%----------------------
% planner structure
planner.action = [-1,0,1]; % with respect to velocity
planner.actionNum = length(planner.action);
planner.actionSetNum = planner.actionNum^clock.nT;
for iAct = 1 : clock.nT
    assignDen = planner.actionSetNum/planner.actionNum^iAct;
    repeatNum = planner.actionSetNum/assignDen;
    for jAct = 1 : repeatNum
        actionElem = planner.action(rem(jAct,planner.actionNum)+1);
        for kAct = 1 : assignDen
            planner.actionSet(iAct,assignDen*(jAct-1)+kAct) = actionElem;
        end
    end
end
planner.actionSetNum = 1;

planner.xhat = PF.xhat;
planner.nPt = PF.nPt;
planner.pt = PF.pt;
planner.w = PF.w;

planner.input = nan(clock.nT,1);
planner.actIdx = nan;
planner.hist.input = planner.input;
planner.hist.actIdx = planner.actIdx;

planner.I = nan;
planner.hist.I = planner.I;
planner.hist.Hbefore = nan(clock.nT,1);
planner.hist.Hafter = nan(clock.nT,1);

planner.param.dRefPt = 1;
planner.param.RefPt = field.boundary(1):planner.param.dRefPt:field.boundary(2);

planner.param.plot.row = clock.nT; 
planner.param.plot.col = 2;
%----------------------


%-----------------------------------
% Linear-Gaussian M.I. computation
%-----------------------------------

for iClock = 1:clock.nt
    
    % agent moving
    agent.pos = agent.pos + agent.vel;
    agent.hist.pos(:,iClock+1) = agent.pos;
    
    % target moving
    target.pos = target.pos + target.vel;
    target.hist.pos(:,iClock+1) = target.pos;
    
    % take measurement
    if target.pos >= agent.pos-sensor.regionRadius && target.pos <= agent.pos+sensor.regionRadius
        sensor.y = binornd(1,sensor.DetectBeta);
    else
        sensor.y = 0;
    end
    sensor.hist.y(:,iClock+1) = sensor.y;
    
    
    %-----------------------------------
    % PF-based Mutual information Computation and decision-making
    %-----------------------------------

    % compute future information with respect to action profiles
    for iActSet = 1 : planner.actionSetNum
        [planner.candidate.Hbefore(:,iActSet),planner.candidate.Hafter(:,iActSet),planner.candidate.I(iActSet)] = ...
            ComputeFutureInformation(planner,agent,sensor,clock,PF,sim,iActSet,iClock);
    end
    
    % direct decision making: maximize mutual information
    [~,planner.actIdx] = max(planner.candidate.I);
    
    % take optimized information data
    planner.input = planner.actionSet(:,planner.actIdx);
    planner.I = planner.candidate.I(planner.actIdx);
    planner.Hbefore = planner.candidate.Hbefore(:,planner.actIdx);
    planner.Hafter = planner.candidate.Hafter(:,planner.actIdx);
    

    %-----------------------------------
    % Actual measurement and estimation: PF
    %-----------------------------------
        
    for iPt = 1:PF.nPt
        % particle state update
        PF.pt(iPt) = PF.F*PF.pt(iPt) + mvnrnd(0,PF.Q)';
        
        % particle measurement update
        if sensor.y == 1 % when sensor does detect
            if PF.pt(iPt) >= agent.pos-sensor.regionRadius && PF.pt(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                PF.w(iPt) = sensor.DetectBeta;
            else
                PF.w(iPt) = 1-sensor.DetectBeta;
            end
        else
            if PF.pt(iPt) >= agent.pos-sensor.regionRadius && PF.pt(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                PF.w(iPt) = 0;
            else
                PF.w(iPt) = 1;
            end
        end
        
    end
    PF.w = PF.w./sum(PF.w);
    
    % resample particle
    for iPt = 1:PF.nPt
        PF.pt(iPt) = PF.pt(find(rand <= cumsum(PF.w),1));
    end
    
    % particle filter info update/store
    PF.xhat = sum(PF.w.*PF.pt')/sum(PF.w);
    PF.hist.pt(:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = PF.xhat;
    
    % update planner initial info from PF results
    planner.xhat = PF.xhat;
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


figure(2)
plot(clock.hist.time,planner.hist.I,'b--','LineWidth',3);
xlabel('time [sec]'); ylabel('utility [sum of M.I.]');
title('Utility Profile');

figure(3)
[timePlanProfile,timeProfile] = meshgrid(1:clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,planner.hist.Hbefore');
surface(timePlanProfile,timeProfile,planner.hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('particle-H[P(x_t|y_{k+1:t-1})]','particle-H[P(x_t|y_{k+1:t})]');
view(3);
title('Entropy Profile');

