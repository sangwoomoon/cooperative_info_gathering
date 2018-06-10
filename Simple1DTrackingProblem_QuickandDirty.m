
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
sim.flagDisp.before = 0;
sim.flagDisp.after = 0;

clock.nt = 10;
clock.dt = 1;
clock.nT = 5; % planning horizon
clock.hist.time = 0;

target.pos = 10;
target.hist.pos = target.pos;

sensor.hist.y(:,1) = nan;

ref.xhat = 20;
ref.Phat = 12^2;
ref.hist.xhat = ref.xhat;
ref.hist.Phat = ref.Phat;

ref.F = 1;
ref.H = 1;
ref.Q = 10^2;
ref.R = 6^2;

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

particle.pdf.I = nan;
particle.hist.pdf.I = particle.pdf.I;
particle.hist.pdf.Hbefore = nan(clock.nT,1);
particle.hist.pdf.Hafter = nan(clock.nT,1);

particle.ryan.I = nan;
particle.hist.ryan.I = particle.ryan.I;
particle.hist.ryan.Hbefore = nan(clock.nT,1);
particle.hist.ryan.Hafter = nan(clock.nT,1);

KF.xhat = ref.xhat;
KF.Phat = ref.Phat;
KF.hist.xhat = KF.xhat;
KF.hist.Phat = KF.Phat;

KF.F = ref.F;
KF.H = ref.H;
KF.Q = ref.Q;
KF.R = ref.R;

param.regressPlot = 1/clock.nT;
param.dRefPt = 10;
param.refPt = -10*sqrt(ref.Phat):param.dRefPt:10*sqrt(ref.Phat);

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
        ref.PhatMinus = ref.F*ref.Phat*ref.F' + ref.Q;
        
        % H(P(x_k|y_{k-1}))        
        ref.Hbefore(iPlan) = 0.5*log((2*pi*exp(1))*det(ref.PhatMinus));

        %-- Checking -----------
        if sim.flagDisp.before == 1
            ref.targetUpdateProb = GenerateGaussianPDF(ref.xhat,ref.PhatMinus,param);
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*(iPlan-1)+1),
            plot(param.refPt,ref.targetUpdateProb,'r-','LineWidth',2); hold on;
        end
        %----------------------
        
        
        % measurement update: P(x_k|y_k)
        ref.K = ref.PhatMinus*ref.H'*(ref.R+ref.H*ref.PhatMinus*ref.H')^(-1);
        ref.Phat = (eye(1)-ref.K*ref.H)*ref.PhatMinus*(eye(1)-ref.K*ref.H)' + ref.K*ref.R*ref.K';
        
        % H(P(x_k|y_k))
        ref.Hafter(iPlan) = 0.5*log((2*pi*exp(1))*det(ref.Phat));
        % mutual information
        ref.I = ref.I + (ref.Hbefore(iPlan) - ref.Hafter(iPlan));

        %-- Checking -----------
        if sim.flagDisp.after == 1
            ref.measuUpdateProb = GenerateGaussianPDF(ref.xhat,ref.Phat,param);
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*iPlan),
            plot(param.refPt,ref.measuUpdateProb,'r-','LineWidth',2); hold on;
        end
        %----------------------

    end
    
    % store entropy and mutual info data
    ref.hist.I(:,iClock+1) = ref.I;
    ref.hist.Hafter(:,iClock+1) = ref.Hafter';
    ref.hist.Hbefore(:,iClock+1) = ref.Hbefore';
    
    
    
    
    %% -----------------------------------
    % PF-based Mutual information Computation: Ryan's approach & PDF-based
    % approach (Mine)
    %-----------------------------------

    particle.pdf.I = 0;
    particle.ryan.I = 0;

    % particle initialization based on actual estimated results
    particle.xhat = KF.xhat;
    particle.Phat = KF.Phat;
    
    particle.w = ones(1,particle.nPt)./particle.nPt;
    for iPt = 1 : particle.nPt
        particle.pt(iPt,1) = KF.xhat + mvnrnd(0,KF.Phat)';
    end
    

    for iPlan = 1:clock.nT
        
        
        % store <x_{k-1},w_{k-1}>: For Ryan's approach
        particle.ryan.ptBefore = particle.pt;
        particle.ryan.wBefore = particle.w;
                
        %-----------------------
        %  PDF-based Approach - probability of target evolution: P(X_k|y_{k-1})
        for iPt = 1:particle.nPt
            onePtTargetUpdateProb = GenerateGaussianPDF(particle.pt(iPt),particle.Q,param);
            
            if iPt == 1
                particle.pdf.targetUpdateProb = particle.w(iPt)*onePtTargetUpdateProb;
            else
                particle.pdf.targetUpdateProb = particle.pdf.targetUpdateProb + particle.w(iPt)*onePtTargetUpdateProb;
            end
            
            % particle state update: x_k ~ P(X_k|X_{k-1})
            particle.pt(iPt) = particle.F*particle.pt(iPt) + mvnrnd(0,particle.Q);
        end
        
        particle.pdf.targetUpdateProb = particle.pdf.targetUpdateProb./(sum(particle.pdf.targetUpdateProb)*param.dRefPt);
        %-----------------------

        %-----------------------
        % PDF approach - Entropy computation: H(X_k|y_{k-1})
        NonZeroIndex = particle.pdf.targetUpdateProb > 0; % to prevent from log(0)
        particle.pdf.Hbefore(iPlan) = -sum(particle.pdf.targetUpdateProb(NonZeroIndex).*log(particle.pdf.targetUpdateProb(NonZeroIndex)).*param.dRefPt);
        %-----------------------
        
        %-- Checking -----------
        if sim.flagDisp.before == 1
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*(iPlan-1)+1),
            plot(param.refPt,particle.pdf.targetUpdateProb,'b-','LineWidth',2);
            plot(particle.pt,particle.w,'m.','LineWidth',3);
            
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
        end
        %-----------------------
        
        %-----------------------
        % P(y_k|y_{k-1})
        for iPt = 1:particle.nPt
            % expected measurement setting
            particle.ypt(iPt) = particle.H*particle.pt(iPt);
            
            onePtMeasProb = GenerateGaussianPDF(particle.ypt(iPt),particle.R,param);
            
            if iPt == 1
                particle.pdf.measProb = particle.w(iPt)*onePtMeasProb;
            else
                particle.pdf.measProb = particle.pdf.measProb + particle.w(iPt)*onePtMeasProb;
            end
        end
        
        particle.pdf.measProb = particle.pdf.measProb./(sum(particle.pdf.measProb)*param.dRefPt);
        %-----------------------
        
        
        % sample measurement: for weight update of PDF approach and
        % likelihood function of Ryan's approach
        particle.yIdx(:,iPlan) = find(rand/param.dRefPt <= cumsum(particle.pdf.measProb),1);
        particle.yProb(:,iPlan) = particle.pdf.measProb(particle.yIdx(:,iPlan));
        particle.ySample(:,iPlan) = param.refPt(particle.yIdx(:,iPlan));
                
        %-----------------------
        % weight update: w_{k-1} -> w_k

        numParticleUpdate = nan(1,particle.nPt);
        for iPt = 1:particle.nPt
            % sampled measurement comparison and weight update
            numParticleUpdate(iPt) = (1/sqrt(2*pi*particle.R)) * exp(-(particle.ySample(:,iPlan) - particle.ypt(iPt))^2/(2*particle.R));
        end
        particle.w = particle.w.*numParticleUpdate;
        particle.w = particle.w./sum(particle.w);
        %-----------------------

        %--------------
        % probability of measurement correction: P(X_k|y_k)        
        particle.pdf.likelihoodProb = GenerateGaussianPDF(particle.ySample(:,iPlan),particle.R,param);
        particle.pdf.measUpdateProb = particle.pdf.likelihoodProb.*particle.pdf.targetUpdateProb;
        particle.pdf.measUpdateProb = particle.pdf.measUpdateProb./(sum(particle.pdf.measUpdateProb)*param.dRefPt);
        %-----------------------

        %-----------------------
        % Entropy computation: H(X_k|Y_k):
        NonZeroIndex = particle.pdf.measUpdateProb > 0; % to prevent from log(0)
        particle.pdf.Hafter(iPlan) = -sum(particle.pdf.measUpdateProb(NonZeroIndex).*log(particle.pdf.measUpdateProb(NonZeroIndex)).*param.dRefPt);
        %--------------

        %-----------------------
        % Ryan's approach
        for iPt = 1:particle.nPt
            % 1. sum of w^j_{k-1}*P(x^i_k|x^j_{k-1})
            for jPt = 1:particle.nPt
                if jPt == 1
                    % target evolution with respect to particle set
                    particle.ryan.plf.before(iPt) = particle.ryan.wBefore(jPt)*...
                        ((1/sqrt(2*pi*particle.Q))*exp(-(particle.ryan.ptBefore(jPt)-particle.pt(iPt))^2/(2*particle.Q)));
                else
                    % target evolution with respect to particle set
                    particle.ryan.plf.before(iPt) = particle.ryan.plf.before(iPt) + particle.ryan.wBefore(jPt)*...
                        ((1/sqrt(2*pi*particle.Q))*exp(-(particle.ryan.ptBefore(jPt)-particle.pt(iPt))^2/(2*particle.Q)));
                end
            end
            % P(z_sample|x^i_k)*{sum of w^j_{k-1}*P(x^i_k|x^j_{k-1})}
            particle.ryan.plf.after(iPt) = particle.ryan.plf.before(iPt)*...
                ((1/sqrt(2*pi*particle.R))*exp(-(particle.ySample(:,iPlan)-particle.ypt(iPt))^2/(2*particle.R)));
        end
        
        % P(z_sample|x^i_k)*{sum of w^j_{k-1}*P(x^i_k|x^j_{k-1})} / P(z_sample|z_{k-1})
        particle.ryan.plf.after = particle.ryan.plf.after./particle.yProb(:,iPlan);
        %-----------------------
        
        %-----------------------
        % Ryan's approach - Entropy computation: H(X_k|y_{k-1})
        particle.ryan.Hbefore(iPlan) = 0;
        
        % sorting <x_sample,P(x_sample)> in order to integrate
        ryanParticleData = sortrows([particle.pt particle.ryan.plf.before']);
        
        for iPt = 2:particle.nPt
            % to avoid particles at the same location
            if (ryanParticleData(iPt-1,1) - ryanParticleData(iPt,1)) ~= 0
                x1 = ryanParticleData(iPt-1,1);
                x2 = ryanParticleData(iPt,1);
                
                p1 = ryanParticleData(iPt-1,2);
                p2 = ryanParticleData(iPt,2);
                
                delta = (p2-p1)/(x2-x1);
                
                value1 = p1^2/2*log(p1)-p1^2/4;
                value2 = p2^2/2*log(p2)-p2^2/4;
                
                particle.ryan.Hbefore(iPlan) = particle.ryan.Hbefore(iPlan) - 1/delta*(value2 - value1);
            end
        end        
        %-----------------------
        
        %-----------------------
        % Ryan's approach - Entropy computation: H(X_k|y_k)
        particle.ryan.Hafter(iPlan) = 0;
        
        % sorting <x_sample,P(x_sample)> in order to integrate
        ryanParticleData = sortrows([particle.pt particle.ryan.plf.after']);
        
        for iPt = 2:particle.nPt
            % to avoid particles at the same location
            if (ryanParticleData(iPt-1,1) - ryanParticleData(iPt,1)) ~= 0
                x1 = ryanParticleData(iPt-1,1);
                x2 = ryanParticleData(iPt,1);
                
                p1 = ryanParticleData(iPt-1,2);
                p2 = ryanParticleData(iPt,2);
                
                delta = (p2-p1)/(x2-x1);
                
                value1 = p1^2/2*log(p1)-p1^2/4;
                value2 = p2^2/2*log(p2)-p2^2/4;
                
                particle.ryan.Hafter(iPlan) = particle.ryan.Hafter(iPlan) - 1/delta*(value2 - value1);
            end
        end
        %-----------------------
   

        %-- Checking -----------
        if sim.flagDisp.after == 1
            figure(iClock+10),subplot(param.planPlot.row,param.planPlot.col,2*iPlan),
            plot(param.refPt,particle.pdf.measUpdateProb,'b-','LineWidth',2);
            plot(param.refPt,particle.pdf.likelihoodProb,'g--','LineWidth',2);
            plot(param.refPt,particle.pdf.measProb,'c--','LineWidth',2);
            plot(particle.pt,particle.ryan.plf.after,'m.','LineWidth',5);
            
            if iPlan == 1
                title('P(x_t|y_{k+1:t})','fontsize',10);
                legend('Ref-PDF','Particle-PDF','P(y_{sample,t}|x_t)','P(y_t|y_{k+1:t-1})','Particle');
            end
        end
        %----------------------

        
        % compute mutual information
        particle.pdf.I = particle.pdf.I + (particle.pdf.Hbefore(iPlan) - particle.pdf.Hafter(iPlan));
        particle.ryan.I = particle.ryan.I + (particle.ryan.Hbefore(iPlan) - particle.ryan.Hafter(iPlan));
        
        % resample particle
        for iPt = 1:particle.nPt
            particle.pt(iPt) = particle.pt(find(rand <= cumsum(particle.w),1));
        end
        particle.w = (1/particle.nPt)*ones(1,particle.nPt);
 
    end
    
    % store entropy and mutual info data: PDF-based approach
    particle.hist.pdf.I(:,iClock+1) = particle.pdf.I;
    particle.hist.pdf.Hafter(:,iClock+1) = particle.pdf.Hafter';
    particle.hist.pdf.Hbefore(:,iClock+1) = particle.pdf.Hbefore';
    
    % store entropy and mutual info data: Ryan approach
    particle.hist.ryan.I(:,iClock+1) = particle.ryan.I;
    particle.hist.ryan.Hbefore(:,iClock+1) = particle.ryan.Hbefore';
    particle.hist.ryan.Hafter(:,iClock+1) = particle.ryan.Hafter';
    
        
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
    fprintf('iClock = %d\n',iClock);
    
end

%% ----------------------------
% Sim Result Plot
%----------------------------

figure(1)
plot(clock.hist.time,KF.hist.xhat-target.hist.pos,'LineWidth',2); hold on;
plot(clock.hist.time,2*sqrt(KF.hist.Phat),'c--','LineWidth',2);
plot(clock.hist.time,-2*sqrt(KF.hist.Phat),'c--','LineWidth',2);
xlabel('time [sec]'); ylabel('estimated target pos error [m]');
title('Actual Estimation (KF)');
legend('\mu error','+2\sigma','-2\sigma');


figure(2)
plot(clock.hist.time,ref.hist.I,'r--','LineWidth',3); hold on;
plot(clock.hist.time,particle.hist.pdf.I,'b-','LineWidth',3);
plot(clock.hist.time,particle.hist.ryan.I,'g-','LineWidth',3);
xlabel('time [sec]'); ylabel('utility [sum of M.I.]');
title('Utility Profile');
legend('true','particle-Moon','particle-Ryan');

figure(3)
[timePlanProfile,timeProfile] = meshgrid(1:clock.nT,clock.hist.time);
mesh(timePlanProfile,timeProfile,ref.hist.Hbefore'); hold on; grid on;
mesh(timePlanProfile,timeProfile,ref.hist.Hafter');
surface(timePlanProfile,timeProfile,particle.hist.pdf.Hbefore');
surface(timePlanProfile,timeProfile,particle.hist.pdf.Hafter');
surface(timePlanProfile,timeProfile,particle.hist.ryan.Hbefore');
surface(timePlanProfile,timeProfile,particle.hist.ryan.Hafter');
xlabel('planned time [sec]'); ylabel('actual time [sec]'); zlabel('entropy');
legend('true-H[P(x_t|y_{k+1:t-1})]','true-H[P(x_t|y_{k+1:t})]','Moon-H[P(x_t|y_{k+1:t-1})]','Moon-H[P(x_t|y_{k+1:t})]','Ryan-H[P(x_t|y_{k+1:t-1})]','Ryan-H[P(x_t|y_{k+1:t})]');
view(3);
title('Entropy Profile');

figure(4)
plot(clock.hist.time,sum(ref.hist.Hbefore,1),'r--','LineWidth',3); hold on;
plot(clock.hist.time,sum(particle.hist.pdf.Hbefore,1),'b-','LineWidth',3);
plot(clock.hist.time,sum(particle.hist.ryan.Hbefore,1),'g-','LineWidth',3);
xlabel('time [sec]'); ylabel('entropy');
title('\Sigma_{k+1}^{k+T} H[P(x_t|y_{k+1:t-1})]');
legend('true','particle-Moon','particle-Ryan');

figure(5)
plot(clock.hist.time,sum(ref.hist.Hafter,1),'r--','LineWidth',3); hold on;
plot(clock.hist.time,sum(particle.hist.pdf.Hafter,1),'b-','LineWidth',3);
plot(clock.hist.time,sum(particle.hist.ryan.Hafter,1),'g-','LineWidth',3);
xlabel('time [sec]'); ylabel('entropy');
title('\Sigma_{k+1}^{k+T} H[P(x_t|y_{k+1:t})]');
legend('true','particle-Moon','particle-Ryan');
