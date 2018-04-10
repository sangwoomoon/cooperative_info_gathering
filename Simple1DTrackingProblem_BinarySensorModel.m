
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
clock.nt = 20;
clock.dt = 1;
clock.nT = 3; % planning horizon
clock.hist.time = 0;
%----------------------

%----------------------
% target structure
target.pos = 0;
target.vel = 0;
target.hist.pos = target.pos;
%----------------------

%----------------------
% agent structure
agent.pos = 5;
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
PF.Phat = 10^2;
PF.hist.xhat = PF.xhat;

PF.F = 1;
PF.Q = 7^2;

PF.nPt = 200;
PF.w = ones(1,PF.nPt)./PF.nPt;
for iPt = 1 : PF.nPt
    PF.pt(iPt,1) = PF.xhat + mvnrnd(0,PF.Phat)';
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

planner.xhat = PF.xhat;
planner.nPt = PF.nPt;
planner.pt = PF.pt;
planner.w = PF.w;

planner.I = nan;
planner.hist.I = planner.I;
planner.hist.Hbefore = nan(clock.nT,1);
planner.hist.Hafter = nan(clock.nT,1);

planner.param.dRefPt = 1;
planner.param.RefPt = -100:planner.param.dRefPt:100;

planner.param.plot.regress = 1/clock.nT;
planner.param.plot.row = 4; 
planner.param.plot.col = floor(clock.nt/planner.param.plot.row)+(mod(clock.nt,planner.param.plot.row) ~= 0)*1;
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
    % PF-based Mutual information Computation
    %-----------------------------------
    
    planner.I = 0;

    for iPlan = 1:clock.nT
        
        % sample measurement
        if planner.xhat >= agent.pos-sensor.regionRadius && planner.xhat <= agent.pos+sensor.regionRadius
            planner.y(:,iPlan) = binornd(1,sensor.DetectBeta);
        else
            planner.y(:,iPlan) = 0;
        end
                
        %--------------
        % Sum of prob. target evolution P(X_k|Y_{k-1})
        onePtTargetProb = nan(1,length(planner.param.RefPt));
        for iPt = 1:planner.nPt
            for iRefpt = 1:length(planner.param.RefPt)
                onePtTargetProb(iRefpt) = (1/sqrt(2*pi*PF.Q))*exp(-(planner.param.RefPt(iRefpt)-planner.pt(iPt))^2/(2*PF.Q));
            end
            
            if iPt == 1
                targetProb = planner.w(iPt)*onePtTargetProb;
            else
                targetProb = targetProb+planner.w(iPt)*onePtTargetProb;
            end
        end
        
        % Entropy computation: H(X_k|Y_{k-1})
        targetProbNorm = targetProb./sum(targetProb);
        NonZeroIndex = targetProbNorm > 0; % to prevent from log(0)
        planner.Hbefore(iPlan) = -sum(targetProbNorm(NonZeroIndex).*log(targetProbNorm(NonZeroIndex)));
        %--------------
        
        %-- Checking -----------
        if sim.flagDisp.before == 1
            figure(10),subplot(planner.param.plot.row,planner.param.plot.col,iClock),
            plot(planner.param.RefPt,targetProb,'-','LineWidth',2,'color',[planner.param.plot.regress*(iPlan-1),1,planner.param.plot.regress*(iPlan-1)]); hold on;
        end
        %-----------------------
        
        
        for iPt = 1:planner.nPt
            % particle state update
            planner.pt(iPt) = PF.F*planner.pt(iPt) + mvnrnd(0,PF.Q)';

            % particle measurement update
            if planner.y(:,iPlan) == 1 % when simulated sensor does detect
                if planner.pt(iPt) >= agent.pos-sensor.regionRadius && planner.pt(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                    planner.w(iPt) = sensor.DetectBeta;
                else
                    planner.w(iPt) = 1-sensor.DetectBeta;
                end
            else
                if planner.pt(iPt) >= agent.pos-sensor.regionRadius && planner.pt(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                    planner.w(iPt) = 0;
                else
                    planner.w(iPt) = 1;
                end
            end
        end
        planner.w = planner.w./sum(planner.w);
        
        
        % resample particle
        for iPt = 1:planner.nPt
            planner.pt(iPt) = planner.pt(find(rand <= cumsum(planner.w),1));
        end
        
        planner.xhat = sum(planner.w.*planner.pt')/sum(planner.w);                
        
        %--------------
        
        
        %--------------        
        % Sum of prob. measurement correction P(X_k|Y_k):
        oneMeasUpdateProb = nan(1,length(planner.param.RefPt));
        for iPt = 1:PF.nPt
            for iRefpt = 1:length(planner.param.RefPt)
                
                if abs(planner.param.RefPt(iRefpt)-planner.pt(iPt)) < planner.param.dRefPt
                    oneMeasUpdateProb(iRefpt) = 1;
                else
                    oneMeasUpdateProb(iRefpt) = 0;
                end
                
            end
            
            if iPt == 1
                measUpdateProb = planner.w(iPt)*oneMeasUpdateProb;
            else
                measUpdateProb = measUpdateProb+planner.w(iPt)*oneMeasUpdateProb;
            end
        end        
        
        % Entropy computation: H(X_k|Y_k):
        measUpdateProbNorm = measUpdateProb./sum(measUpdateProb);
        NonZeroIndex = measUpdateProbNorm > 0; % to prevent from log(0)
        planner.Hafter(iPlan) = -sum(measUpdateProbNorm(NonZeroIndex).*log(measUpdateProbNorm(NonZeroIndex)));
        
        % Mutual Information computation and accumulation for getting cost
        planner.I = planner.I + (planner.Hbefore(iPlan) - planner.Hafter(iPlan));

        %--------------

        %-- Checking -----------
        if sim.flagDisp.after == 1
            % fprintf('PF-Hafter  @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan,PF.Hafter(iPlan));
            figure(10),subplot(planner.param.plot.row,planner.param.plot.col,iClock),
            plot(planner.param.RefPt,measUpdateProbNorm,'-','LineWidth',1,'color',[planner.param.plot.regress*(iPlan-1),planner.param.plot.regress*(iPlan-1),1]); hold on;
            if iPlan == clock.nT
                title(['Time Step = ',num2str(iClock)],'fontsize',10);
            end
        end
        %----------------------

        
    end
    
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
    
    planner.w = PF.w;
    planner.pt = PF.pt;
    
    PF.xhat = sum(PF.w.*PF.pt')/sum(PF.w);
    PF.hist.pt(:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = PF.xhat;
    
    planner.hist.I(:,iClock+1) = planner.I;
    planner.hist.Hafter(:,iClock+1) = planner.Hafter';
    planner.hist.Hbefore(:,iClock+1) = planner.Hbefore';
    
    % clock update
    clock.hist.time(:,iClock+1) = iClock*clock.dt;
    
end

%----------------------------
% Sim Result Plot
%----------------------------

figure(1)
for iClock = 1 : clock.nt
    plot(clock.hist.time(iClock)*ones(1,PF.nPt),PF.hist.pt(:,iClock),'m.','LineWidth',2);
end
plot(clock.hist.time,target.hist.pos,'c--','Linewidth',2);
plot(clock.hist.time,PF.hist.xhat,'r-','Linewidth',2);
xlabel('time [sec]'); ylabel('Target Pos [m]');

figure(2)
plot(clock.hist.time,planner.hist.I,'b--','LineWidth',3);
xlabel('time [sec]'); ylabel('Utility [sum of M.I.]');

figure(3)
[timePlanProfile,timeProfile] = meshgrid(1:clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,planner.hist.Hbefore'); hold on;
mesh(timePlanProfile,timeProfile,planner.hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('PF-H_{t-1}','PF-H_{t}');
view(3);

figure(4)
surface(timePlanProfile,timeProfile,planner.hist.Hbefore'-planner.hist.Hafter');
colormap(pink); grid on;
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('mutual info');
view(3);

% figure(3)
% for iPlan = 1 : clock.nT
%     plot(clock.hist.time,PF.hist.Hbefore(iPlan,:),'--','LineWidth',2,'color',[1,param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]); hold on;
%     plot(clock.hist.time,PF.hist.Hafter(iPlan,:),'-','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]);
%     xlabel('time [sec]'); ylabel('entropy');
% end