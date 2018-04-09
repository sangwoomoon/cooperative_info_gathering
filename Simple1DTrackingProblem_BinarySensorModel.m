
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

sim.nAgent = 1;
sim.nTarget = 1;
sim.flagDisp.before = 1;
sim.flagDisp.after = 1;

clock.nt = 20;
clock.dt = 1;
clock.nT = 3; % planning horizon
clock.hist.time = 0;

target.pos = 0;
target.hist.pos = target.pos;

agent.pos = 10;
agent.vel = 0;
agent.hist.pos = agent.pos;

sensor.regionRadius = 10; % sensing region radius
sensor.DetectBeta = 0.85; % bernoulli detection parameter
sensor.hist.y(:,1) = nan;

PF.xhat = 40;
PF.Phat = 10^2;
PF.xhatPlan = PF.xhat;
PF.hist.xhat = PF.xhat;

PF.F = 1;
PF.Q = 7^2;

PF.nPt = 200;
PF.w = ones(1,PF.nPt)./PF.nPt;
for iPt = 1 : PF.nPt
    PF.pt(iPt,1) = PF.xhat + mvnrnd(0,PF.Phat)';
end
PF.hist.pt = PF.pt;
PF.ptPlan = PF.pt;
PF.wPlan = PF.w;

PF.I = nan;
PF.hist.I = PF.I;
PF.hist.Hbefore = nan(clock.nT,1);
PF.hist.Hafter = nan(clock.nT,1);

param.regressPlot = 0.4;
param.dRefPt = 1;
param.RefPt = -100:param.dRefPt:100;
param.planPlot.row = 4; 
param.planPlot.col = floor(clock.nt/param.planPlot.row)+(mod(clock.nt,param.planPlot.row) ~= 0)*1;


%-----------------------------------
% Linear-Gaussian M.I. computation
%-----------------------------------

for iClock = 1:clock.nt
    
    % agent moving
    agent.pos = agent.pos + agent.vel;
    agent.hist.pos(:,iClock+1) = agent.pos;
    
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
    
    PF.I = 0;

    for iPlan = 1:clock.nT
        
        % sample measurement
        if PF.xhatPlan >= agent.pos-sensor.regionRadius && PF.xhatPlan <= agent.pos+sensor.regionRadius
            PF.yPlan(:,iPlan) = binornd(1,sensor.DetectBeta);
        else
            PF.yPlan(:,iPlan) = 0;
        end
                
        %--------------
        % Sum of prob. target evolution P(X_k|Y_{k-1})
        onePtTargetProb = nan(1,length(param.RefPt));
        for iPt = 1:PF.nPt
            for iRefpt = 1:length(param.RefPt)
                onePtTargetProb(iRefpt) = (1/sqrt(2*pi*PF.Q))*exp(-(param.RefPt(iRefpt)-PF.ptPlan(iPt))^2/(2*PF.Q));
            end
            
            if iPt == 1
                PF.targetProb = PF.wPlan(iPt)*onePtTargetProb;
            else
                PF.targetProb = PF.targetProb+PF.wPlan(iPt)*onePtTargetProb;
            end
        end
        
        % Entropy computation: H(X_k|Y_{k-1})
        PF.targetProbNorm = PF.targetProb./sum(PF.targetProb);
        NonZeroIndex = PF.targetProbNorm > 0; % to prevent from log(0)
        PF.Hbefore(iPlan) = -sum(PF.targetProbNorm(NonZeroIndex).*log(PF.targetProbNorm(NonZeroIndex)));
        %--------------
        
        %-- Checking -----------
        if sim.flagDisp.before == 1
            figure(10),subplot(param.planPlot.row,param.planPlot.col,iClock),
            plot(param.RefPt,PF.targetProb,'.','LineWidth',2,'color',[param.regressPlot*(iPlan-1),1,param.regressPlot*(iPlan-1)]); hold on;
            % fprintf('PF-Hbefore @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan,PF.Hbefore(iPlan));
        end
        %-----------------------
        
        
        for iPt = 1:PF.nPt
            % particle state update
            PF.ptPlan(iPt) = PF.F*PF.ptPlan(iPt) + mvnrnd(0,PF.Q)';

            % particle measurement update
            if PF.yPlan(:,iPlan) == 1 % when sensor does detect
                if PF.ptPlan(iPt) >= agent.pos-sensor.regionRadius && PF.ptPlan(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                    PF.wPlan(iPt) = sensor.DetectBeta;
                else
                    PF.wPlan(iPt) = 1-sensor.DetectBeta;
                end
            else
                if PF.ptPlan(iPt) >= agent.pos-sensor.regionRadius && PF.ptPlan(iPt) <= agent.pos+sensor.regionRadius % when the particle is within detected region
                    PF.wPlan(iPt) = 0;
                else
                    PF.wPlan(iPt) = 1;
                end
            end
        end
        PF.wPlan = PF.wPlan./sum(PF.wPlan);
        
        
        % resample particle
        for iPt = 1:PF.nPt
            PF.ptPlan(iPt) = PF.ptPlan(find(rand <= cumsum(PF.wPlan),1));
        end
        
        PF.xhatPlan = sum(PF.wPlan.*PF.ptPlan')/sum(PF.wPlan);                
        
        %--------------
        
        
        %--------------        
        % Sum of prob. measurement correction P(X_k|Y_k):
        oneMeasUpdateProb = nan(1,length(param.RefPt));
        for iPt = 1:PF.nPt
            for iRefpt = 1:length(param.RefPt)
                
                if abs(param.RefPt(iRefpt)-PF.ptPlan(iPt)) < param.dRefPt
                    oneMeasUpdateProb(iRefpt) = 1;
                else
                    oneMeasUpdateProb(iRefpt) = 0;
                end
            end
            
            if iPt == 1
                PF.measUpdateProb = PF.wPlan(iPt)*oneMeasUpdateProb;
            else
                PF.measUpdateProb = PF.measUpdateProb+PF.wPlan(iPt)*oneMeasUpdateProb;
            end
        end        
        
        % Entropy computation: H(X_k|Y_k):
        PF.measUpdateProbNorm = PF.measUpdateProb./sum(PF.measUpdateProb);
        NonZeroIndex = PF.measUpdateProbNorm > 0; % to prevent from log(0)
        PF.Hafter(iPlan) = -sum(PF.measUpdateProbNorm(NonZeroIndex).*log(PF.measUpdateProbNorm(NonZeroIndex)));
        
        % Mutual Information computation and accumulation for getting cost
        PF.I = PF.I + (PF.Hbefore(iPlan) - PF.Hafter(iPlan));

        %--------------

        %-- Checking -----------
        if sim.flagDisp.after == 1
            % fprintf('PF-Hafter  @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan,PF.Hafter(iPlan));
            figure(10),subplot(param.planPlot.row,param.planPlot.col,iClock),
            plot(param.RefPt,PF.measUpdateProbNorm,'-','LineWidth',1,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]); hold on;
            if iPlan == clock.nT
                title(['Time Step = ',num2str(iClock)]);
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
    
    PF.wPlan = PF.w;
    PF.ptPlan = PF.pt;
    
    PF.xhat = sum(PF.w.*PF.pt')/sum(PF.w);
    PF.hist.pt(:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = PF.xhat;
    PF.hist.I(:,iClock+1) = PF.I;
    PF.hist.Hafter(:,iClock+1) = PF.Hafter';
    PF.hist.Hbefore(:,iClock+1) = PF.Hbefore';
    
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
plot(clock.hist.time,PF.hist.xhat,'r-','Linewidth',2);
xlabel('time [sec]'); ylabel('Target Pos [m]');

figure(2)
plot(clock.hist.time,PF.hist.I,'b--','LineWidth',3);
xlabel('time [sec]'); ylabel('Utility [sum of M.I.]');

figure(3)
for iPlan = 1 : clock.nT
    plot(clock.hist.time,PF.hist.Hbefore(iPlan,:),'--','LineWidth',2,'color',[1,param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]); hold on;
    plot(clock.hist.time,PF.hist.Hafter(iPlan,:),'-','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]);
    xlabel('time [sec]'); ylabel('entropy');
end