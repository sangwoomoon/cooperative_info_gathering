
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

clock.nt = 3;
clock.dt = 1;
clock.nT = 5; % planning horizon
clock.hist.time = 0;

target.pos = 0;
target.hist.pos = target.pos;

sensor.hist.y(:,1) = nan;

ref.xhat = 20;
ref.Phat = 50;
ref.hist.xhat = ref.xhat;
ref.hist.Phat = ref.Phat;

ref.F = 1;
ref.H = 1;
ref.Q = 10^2;
ref.R = 7^2;

ref.I = nan;
ref.hist.I = ref.I;
ref.hist.Hbefore = nan(clock.nT,1);
ref.hist.Hafter = nan(clock.nT,1);


particle.xhat = ref.xhat;
particle.Phat = ref.Phat;
particle.xhatPlan = particle.xhat;
particle.hist.xhat = particle.xhat;

particle.F = 1;
particle.H = 1;
particle.Q = ref.Q;
particle.R = ref.R;

particle.nPt = 200;

particle.I = nan;
particle.hist.I = particle.I;
particle.hist.Hbefore = nan(clock.nT,1);
particle.hist.Hafter = nan(clock.nT,1);


KF.xhat = ref.xhat;
KF.Phat = ref.Phat;
KF.hist.xhat = KF.xhat;
KF.hist.Phat = KF.Phat;

KF.F = ref.F;
KF.H = ref.H;
KF.Q = ref.Q;
KF.R = ref.R;

param.regressPlot = 1/clock.nT;
param.dRefPt = 1;
param.RefPt = -100:param.dRefPt:100;
param.direcDeltaVar = 1^2;

param.planPlot.row = clock.nT; 
param.planPlot.col = 2;

%-----------------------------------
% Linear-Gaussian M.I. computation
%-----------------------------------

for iClock = 1:clock.nt
        
    %% -----------------------------------
    % closed loop estimation-based Mutual information for Linear-Gaussian M.I. computation
    %-----------------------------------
    
    ref.I = 0;
    
    % reference initialization based on actual estimated results
    ref.xhat = KF.xhat;
    ref.Phat = KF.Phat;
    
    for iPlan = 1:clock.nT
        
        % target evolution: P(x_k|x_{k-1})
        ref.xhat = ref.F*ref.xhat;
        ref.Phat = ref.F*ref.Phat*ref.F' + ref.Q;
        
        % H(P(x_k|y_{k-1}))        
        ref.Hbefore(iPlan) = 0.5*log((2*pi*exp(1))*det(ref.Phat));

        %-- Checking -----------
        if sim.flagDisp.before == 1
            ref.targetUpdateProb = GenerateGaussianPDF(ref.xhat,ref.Phat,param);
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*(iPlan-1)+1),
            plot(param.RefPt,ref.targetUpdateProb,'r-','LineWidth',2); hold on;
        end
        %----------------------
        
        
        % measurement update: P(x_k|y_k)
        ref.K = ref.Phat*ref.H'*(ref.R+ref.H*ref.Phat*ref.H')^(-1);
        ref.Phat = (eye(1)-ref.K*ref.H)*ref.Phat*(eye(1)-ref.K*ref.H)' + ref.K*ref.R*ref.K';
        
        % H(P(x_k|y_k))
        ref.Hafter(iPlan) = 0.5*log((2*pi*exp(1))*det(ref.Phat));
        % mutual information
        ref.I = ref.I + (ref.Hbefore(iPlan) - ref.Hafter(iPlan));

        %-- Checking -----------
        if sim.flagDisp.after == 1
            ref.measuUpdateProb = GenerateGaussianPDF(ref.xhat,ref.Phat,param);
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*iPlan),
            plot(param.RefPt,ref.measuUpdateProb,'r-','LineWidth',2); hold on;
        end
        %----------------------

    end
    
    % store entropy and mutual info data
    ref.hist.I(:,iClock+1) = ref.I;
    ref.hist.Hafter(:,iClock+1) = ref.Hafter';
    ref.hist.Hbefore(:,iClock+1) = ref.Hbefore';
    
    
    
    
    %% -----------------------------------
    % PF-based Mutual information Computation
    %-----------------------------------

    particle.I = 0;

    % particle initialization based on actual estimated results
    particle.xhat = KF.xhat;
    particle.Phat = KF.Phat;
    
    particle.w = ones(1,particle.nPt)./particle.nPt;
    for iPt = 1 : particle.nPt
        particle.pt(iPt,1) = KF.xhat + mvnrnd(0,KF.Phat)';
    end
    

    for iPlan = 1:clock.nT
                
        %--------------
        % probability of target evolution: P(X_k|y_{k-1})
        onePtTargetProb = nan(1,length(param.RefPt));
        for iPt = 1:particle.nPt
            onePtTargetUpdateProb = GenerateGaussianPDF(particle.pt(iPt),particle.Q,param);
     
            if iPt == 1
                particle.targetUpdateProb = particle.w(iPt)*onePtTargetUpdateProb;
            else
                particle.targetUpdateProb = particle.targetUpdateProb + particle.w(iPt)*onePtTargetUpdateProb;
            end
        end
        particle.targetUpdateProb = particle.targetUpdateProb./sum(particle.targetUpdateProb);
        %-----------------------

        %-----------------------
        % Entropy computation: H(X_k|y_{k-1})
        NonZeroIndex = particle.targetUpdateProb > 0; % to prevent from log(0)
        particle.Hbefore(iPlan) = -sum(particle.targetUpdateProb(NonZeroIndex).*log(particle.targetUpdateProb(NonZeroIndex)));
        %--------------
        
        %-- Checking -----------
        if sim.flagDisp.before == 1
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*(iPlan-1)+1),
            plot(param.RefPt,particle.targetUpdateProb,'b-','LineWidth',2);
            plot(particle.pt,zeros(1,particle.nPt),'m.','LineWidth',3);
        end
        
        if iPlan == 1
            title('P(x_t|y_{k+1:t-1})','fontsize',10);
        end
        ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
        %-----------------------
        
        %-----------------------
        % weight update: w_{k-1} -> w_k
        
        % sample measurement: for weight update
        particle.y(:,iPlan) = particle.H*particle.xhat;

        numParticleUpdate = nan(1,particle.nPt);
        for iPt = 1:particle.nPt
            % expected particle state update
            particle.pt(iPt) = particle.F*particle.pt(iPt);
            
            % expected measurement setting
            particle.ypt(iPt) = particle.H*particle.pt(iPt);
            
            % sampled measurement comparison and weight update
            numParticleUpdate(iPt) = (1/sqrt(2*pi*particle.R)) * exp(-(particle.y(:,iPlan) - particle.ypt(iPt))^2/(2*particle.R));
        end
        denParticleUpdate = sum(numParticleUpdate);
        particle.w = particle.w.*numParticleUpdate/denParticleUpdate';
        particle.w = particle.w./sum(particle.w);
        %-----------------------

        %--------------
        % probability of measurement correction: P(X_k|y_k)        
        particle.likelihoodProb = GenerateGaussianPDF(particle.y(:,iPlan),particle.R,param);
        particle.measUpdateProb = particle.likelihoodProb.*particle.targetUpdateProb;
        particle.measUpdateProb = particle.measUpdateProb./sum(particle.measUpdateProb);
        %-----------------------

        %-----------------------
        % Entropy computation: H(X_k|Y_k):
        NonZeroIndex = particle.measUpdateProb > 0; % to prevent from log(0)
        particle.Hafter(iPlan) = -sum(particle.measUpdateProb(NonZeroIndex).*log(particle.measUpdateProb(NonZeroIndex)));
        %--------------
        
        %-- Checking -----------
        if sim.flagDisp.after == 1
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*iPlan),
            plot(param.RefPt,particle.measUpdateProb,'b-','LineWidth',2);
            plot(param.RefPt,particle.likelihoodProb,'g--','LineWidth',2);
            plot(particle.pt,zeros(1,particle.nPt),'m.','LineWidth',3);
        end
        
        if iPlan == 1
            title('P(x_t|y_{k+1:t})','fontsize',10);
            legend('Ref-PDF','Particle-PDF','Likelihood, P(y_t|x_t)','Particle'); 
        end
        %----------------------
        
        
        % resample particle
        for iPt = 1:particle.nPt
            particle.pt(iPt) = particle.pt(find(rand <= cumsum(particle.w),1));
        end
        particle.w = (1/particle.nPt)*ones(1,particle.nPt);
        particle.xhat = sum(particle.w.*particle.pt')/sum(particle.w); % should be the same
        
        % compute mutual information
        particle.I = particle.I + (particle.Hbefore(iPlan) - particle.Hafter(iPlan));
        
 
    end
    
    % store entropy and mutual info data
    particle.hist.I(:,iClock+1) = particle.I;
    particle.hist.Hafter(:,iClock+1) = particle.Hafter';
    particle.hist.Hbefore(:,iClock+1) = particle.Hbefore';
    
        
    %% -----------------------------------
    % Actual measurement and estimation: KF
    %-----------------------------------
    
    % take measurement
    sensor.y = target.pos + mvnrnd(0,KF.R)';
    sensor.hist.y(:,iClock+1) = sensor.y;
    
    % state update
    KF.xhat = KF.F*KF.xhat;
    KF.Phat = KF.F*KF.Phat*KF.F' + KF.Q;
    
    % measurement update
    KF.K = KF.Phat*KF.H'*(KF.R+KF.H*KF.Phat*KF.H')^(-1);
    KF.xhat = KF.xhat + KF.K*(sensor.y - KF.H*KF.xhat);
    KF.Phat = (eye(1)-KF.K*KF.H)*KF.Phat*(eye(1)-KF.K*KF.H)' + KF.K*KF.R*KF.K';
    
    % store estimation data
    KF.hist.xhat(:,iClock+1) = KF.xhat;
    KF.hist.Phat(:,iClock+1) = KF.Phat;
    
    
    %% clock update
    clock.hist.time(:,iClock+1) = iClock*clock.dt;
    
end

%% ----------------------------
% Sim Result Plot
%----------------------------

figure(1)
plot(clock.hist.time,KF.hist.xhat,'LineWidth',2); hold on;
plot(clock.hist.time,KF.hist.xhat+2*sqrt(KF.hist.Phat),'c--','LineWidth',2);
plot(clock.hist.time,KF.hist.xhat-2*sqrt(KF.hist.Phat),'c--','LineWidth',2);
xlabel('time [sec]'); ylabel('estimated target pos [m]');
title('Actual Estimation (KF)');
legend('\mu','\mu+2\sigma','\mu-2\sigma');


figure(2)
plot(clock.hist.time,ref.hist.I,'r--','LineWidth',3); hold on;
plot(clock.hist.time,particle.hist.I,'b--','LineWidth',3);
xlabel('time [sec]'); ylabel('utility [sum of M.I.]');
title('Utility Profile');
legend('ref','particle');

figure(3)
[timePlanProfile,timeProfile] = meshgrid(1:clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,ref.hist.Hbefore'); hold on; grid on;
mesh(timePlanProfile,timeProfile,ref.hist.Hafter');
surface(timePlanProfile,timeProfile,particle.hist.Hbefore');
surface(timePlanProfile,timeProfile,particle.hist.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('ref-H[P(x_t|y_{k+1:t-1})]','ref-H[P(x_t|y_{k+1:t})]','particle-H[P(x_t|y_{k+1:t-1})]','particle-H[P(x_t|y_{k+1:t})]');
view(3);
title('Entropy Profile');

