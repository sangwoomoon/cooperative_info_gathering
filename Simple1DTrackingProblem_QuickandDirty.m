
% Single Agent - Single Target Localization Problem
%
% X(t+1) = X(t) + W (linear Gaussian, stationary)
% Y(t) = X(t) + V (linear Gaussian, directly measures target state)
% Particle filter based Mutual Information Checker
% 
% - Quick and Dirty Ver.
% - coded by Sangwoo Moon
% - Created: 3/19/2018


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

clock.nt = 5;
clock.dt = 1;
clock.nT = 3; % planning horizon
clock.hist.time = 0;

target.pos = 0;
target.hist.pos = target.pos;

sensor.hist.y(:,1) = nan;

KF.xhat = 20;
KF.Phat = 50;
KF.xhatPlan = KF.xhat;
KF.PhatPlan = KF.Phat;
KF.hist.xhat = KF.xhat;
KF.hist.Phat = KF.Phat;

KF.F = 1;
KF.H = 1;
KF.Q = 5^2;
KF.R = 2^2;

KF.I = nan;
KF.hist.I = KF.I;
KF.hist.Hbefore = nan(clock.nT,1);
KF.hist.Hafter = nan(clock.nT,1);

PF.xhat = KF.xhat;
PF.Phat = KF.Phat;
PF.xhatPlan = PF.xhat;
PF.hist.xhat = PF.xhat;

PF.F = 1;
PF.H = 1;
PF.Q = KF.Q;
PF.R = KF.R;

PF.nPt = 10000;
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
    
    % take measurement
    sensor.y = target.pos + mvnrnd(0,KF.R)';
    sensor.hist.y(:,iClock+1) = sensor.y;
    
    %-----------------------------------
    % KF-based Mutual information for Linear-Gaussian M.I. computation
    %-----------------------------------
    
    KF.I = 0;
    
    for iPlan = 1:clock.nT
        
        % sample measurement
        KF.yPlan(:,iPlan) = KF.H*KF.xhatPlan + mvnrnd(0,KF.R)';
        
        % state update: P(x_k|x_{k-1})
        KF.xhatPlan = KF.F*KF.xhatPlan;
        KF.PhatPlan = KF.F*KF.PhatPlan*KF.F' + KF.Q;
        
        KF.Hbefore(iPlan) = 0.5*log((2*pi*exp(1))*det(KF.PhatPlan));

        %-- Checking -----------
        if sim.flagDisp.before == 1
            for iRefpt = 1:length(param.RefPt)
                TaEvolve(iRefpt) = (1/sqrt(2*pi*KF.PhatPlan))*exp(-(param.RefPt(iRefpt)-KF.xhatPlan)^2/(2*KF.PhatPlan));
            end
            figure(10),subplot(param.planPlot.row,param.planPlot.col,iClock),
            plot(param.RefPt,TaEvolve,'--','LineWidth',2,'color',[1,param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]); hold on;
%             fprintf('KF-Hbefore @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan, KF.Hbefore(iPlan));
        end
        %----------------------
        
        
      % measurement update: FOR KALMAN FILTER PART
        KF.K = KF.PhatPlan*KF.H'*(KF.R+KF.H*KF.PhatPlan*KF.H')^(-1);
        KF.xhatPlan = KF.xhatPlan + KF.K*(KF.yPlan(:,iPlan) - KF.H*KF.xhat);
        KF.PhatPlan = (eye(1)-KF.K*KF.H)*KF.PhatPlan*(eye(1)-KF.K*KF.H)' + KF.K*KF.R*KF.K';

        KF.Hafter(iPlan) = 0.5*log((2*pi*exp(1))*det(KF.PhatPlan));
        KF.I = KF.I + (KF.Hbefore(iPlan) - KF.Hafter(iPlan));

        %-- Checking -----------
        if sim.flagDisp.after == 1
%             fprintf('KF-Hafter  @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan, KF.Hafter(iPlan));
            
            for iRefpt = 1:length(param.RefPt)
                MeasUpdate(iRefpt) = (1/sqrt(2*pi*KF.PhatPlan))*exp(-(param.RefPt(iRefpt)-KF.xhatPlan)^2/(2*KF.PhatPlan));
            end
            figure(11),subplot(param.planPlot.row,param.planPlot.col,iClock),
            plot(param.RefPt,MeasUpdate,'-','LineWidth',2,'color',[1,param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]); hold on;
        end
        %----------------------

    end
    
    %-----------------------------------
    % Actual measurement and estimation: KF
    %-----------------------------------
    
    % state update
    KF.xhat = KF.F*KF.xhat;
    KF.Phat = KF.F*KF.Phat*KF.F' + KF.Q;
    
    % measurement update
    KF.K = KF.Phat*KF.H'*(KF.R+KF.H*KF.Phat*KF.H')^(-1);
    KF.xhat = KF.xhat + KF.K*(sensor.y - KF.H*KF.xhat);
    KF.Phat = (eye(1)-KF.K*KF.H)*KF.Phat*(eye(1)-KF.K*KF.H)' + KF.K*KF.R*KF.K';
    
    KF.hist.xhat(:,iClock+1) = KF.xhat;
    KF.hist.Phat(:,iClock+1) = KF.Phat;
    KF.hist.I(:,iClock+1) = KF.I;
    KF.hist.Hafter(:,iClock+1) = KF.Hafter';
    KF.hist.Hbefore(:,iClock+1) = KF.Hbefore';
    
    KF.xhatPlan = KF.xhat;
    KF.PhatPlan = KF.Phat;
    
    
    %-----------------------------------
    % PF-based Mutual information Computation
    %-----------------------------------
    
    PF.I = 0;

    for iPlan = 1:clock.nT
        
        % sample measurement: sanme condition as KF prediction
        PF.yPlan(:,iPlan) = KF.yPlan(:,iPlan);
                
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
            plot(param.RefPt,PF.targetProb,'--','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]);
%             fprintf('PF-Hbefore @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan,PF.Hbefore(iPlan));
        end
        
        if iPlan == clock.nT
            title(['Time Step = ',num2str(iClock)]);
        end
        %-----------------------
        
        
        for iPt = 1:PF.nPt
            % particle state update
            PF.ptPlan(iPt) = PF.F*PF.ptPlan(iPt) + mvnrnd(0,PF.Q)';
        end
        
        for iPt = 1:PF.nPt
            % particle measurement update
            PF.yptPlan(iPt) = PF.H*PF.ptPlan(iPt) + mvnrnd(0,PF.R)';
            
            % sampled measurement comparison and weight priori
            PF.wPlan(iPt) = (1/sqrt(2*pi*PF.R)) * exp(-(PF.yPlan(:,iPlan) - PF.yptPlan(iPt))^2/(2*PF.R));
        end
        PF.wPlan = PF.wPlan./sum(PF.wPlan);
        
        
        % resample particle
        for iPt = 1:PF.nPt
            PF.ptPlan(iPt) = PF.ptPlan(find(rand <= cumsum(PF.wPlan),1));
        end
        
        %--------------
        % Sum of prob. measurement correction P(X_k|Y_k)
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
        %--------------
        
        PF.xhatPlan = sum(PF.wPlan.*PF.ptPlan')/sum(PF.wPlan);
        PF.I = PF.I + (PF.Hbefore(iPlan) - PF.Hafter(iPlan));
    
        
        %-- Checking -----------
        if sim.flagDisp.after == 1
%             fprintf('PF-Hafter  @ iClock: %2.0d, iPlan: %2.0d = %2.6f\n',iClock, iPlan,PF.Hafter(iPlan));
            figure(11),subplot(param.planPlot.row,param.planPlot.col,iClock),
            plot(param.RefPt,PF.measUpdateProbNorm,'-','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]);
        end
        
        if iPlan == clock.nT
            title(['Time Step = ',num2str(iClock)]);
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
        PF.ypt(iPt) = PF.H*PF.pt(iPt) + mvnrnd(0,PF.R)';
        
        % sampled measurement comparison and weight priori
        PF.w(iPt) = (1/sqrt(2*pi*PF.R)) * exp(-(sensor.y - PF.ypt(iPt))^2/(2*PF.R));
    end
    PF.w = PF.w./sum(PF.w);
    
    % resample particle
    for iPt = 1:PF.nPt
        PF.pt(iPt) = PF.pt(find(rand <= cumsum(PF.w),1));
    end
    
    PF.wPlan = PF.w;
    PF.ptPlan = PF.pt;
    
    PF.hist.pt(:,iClock+1) = PF.pt;  
    PF.hist.xhat(:,iClock+1) = sum(PF.w.*PF.pt')/sum(PF.w);
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
plot(clock.hist.time,KF.hist.xhat,'LineWidth',2); hold on;
plot(clock.hist.time,KF.hist.xhat+3*sqrt(KF.hist.Phat),'c--','LineWidth',2);
plot(clock.hist.time,KF.hist.xhat-3*sqrt(KF.hist.Phat),'c--','LineWidth',2);
xlabel('time [sec]'); ylabel('Target Pos [m]');

for iClock = 1 : clock.nt
    plot(clock.hist.time(iClock)*ones(1,PF.nPt),PF.hist.pt(:,iClock),'m.','LineWidth',2);
end
plot(clock.hist.time,PF.hist.xhat,'r-','Linewidth',2);


figure(2)
plot(clock.hist.time,KF.hist.I,'r--','LineWidth',3); hold on;
plot(clock.hist.time,PF.hist.I,'b--','LineWidth',3);
xlabel('time [sec]'); ylabel('cost [sum of M.I.]');


figure(3)
for iPlan = 1 : clock.nT
    plot(clock.hist.time,KF.hist.Hbefore(iPlan,:),'--','LineWidth',2,'color',[1, param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]); hold on;
    plot(clock.hist.time,KF.hist.Hafter(iPlan,:),'-','LineWidth',2,'color',[1, param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1)]);
    plot(clock.hist.time,PF.hist.Hbefore(iPlan,:),'--','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]); hold on;
    plot(clock.hist.time,PF.hist.Hafter(iPlan,:),'-','LineWidth',2,'color',[param.regressPlot*(iPlan-1),param.regressPlot*(iPlan-1),1]);
    xlabel('time [sec]'); ylabel('entropy');
end