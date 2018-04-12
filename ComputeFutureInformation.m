%-----------------------------------
% PF-based Mutual information Computation
%-----------------------------------
function [Hbefore,Hafter,I] = ComputeFutureInformation(planner,agent,sensor,clock,PF,sim,iAct,iClock)


I = 0;

for iPlan = 1:clock.nT
    
    % agent moving along with planner action
    agent.pos = agent.pos + planner.actionSet(iPlan,iAct);
    
    % sample measurement
    if planner.xhat >= agent.pos-sensor.regionRadius && planner.xhat <= agent.pos+sensor.regionRadius
        planner.y(:,iPlan) = binornd(1,sensor.DetectBeta);
    else
        planner.y(:,iPlan) = 0;
    end
    
    %--------------
    % Sum of prob. target evolution P(X_k|y_{k-1})
    for iPt = 1:planner.nPt
        onePtTargetUpdateProb = GenerateGaussianPDF(planner.pt(iPt),PF.Q,planner.param);
        
        if iPt == 1
            targetUpdateProb = planner.w(iPt)*onePtTargetUpdateProb;
        else
            targetUpdateProb = targetUpdateProb+planner.w(iPt)*onePtTargetUpdateProb;
        end
    end
    
    %-----------------------
    % Entropy computation: H(X_k|y_{k-1})
    targetProbNorm = targetUpdateProb./sum(targetUpdateProb);
    NonZeroIndex = targetProbNorm > 0; % to prevent from log(0)
    Hbefore(iPlan) = -sum(targetProbNorm(NonZeroIndex).*log(targetProbNorm(NonZeroIndex)));
    %--------------
    
    %-- Checking -----------
    if sim.flagDisp.before == 1
        figure(iClock+10),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
        plot(planner.param.RefPt,targetUpdateProb,'b-','LineWidth',2); hold on;
        plot(planner.pt,zeros(1,planner.nPt),'m.','LineWidth',3);
    end
    
    if iPlan == 1
        title('P(x_t|y_{k+1:t-1})','fontsize',10);
    end
    ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
    %-----------------------
    
    %-----------------------
    % weight update: w_{k-1} -> w_k
    
    for iPt = 1:planner.nPt
        % expected particle state with zero process noise
        planner.pt(iPt) = PF.F*planner.pt(iPt);
        
        planner.w(iPt) = OneDimBinarySensorModel(planner.y(:,iPlan),sensor,agent.pos,planner.pt(iPt));
    end
    planner.w = planner.w./sum(planner.w);
    %-----------------------
    
    
    %-----------------------
    % probability of measurement correction P(X_k|Y_k):
    
    % take likelihood function P(Y_k|X_k)
    likelihoodProb = nan(1,length(planner.param.RefPt));
    for iRefpt = 1:length(planner.param.RefPt)
        likelihoodProb(iRefpt) = OneDimBinarySensorModel(planner.y(:,iPlan),sensor,agent.pos,planner.param.RefPt(iRefpt));
    end
    
    measUpdateProb = likelihoodProb.*targetUpdateProb;
    measUpdateProb = measUpdateProb./sum(measUpdateProb);
    %-----------------------

    %-----------------------
    % Entropy computation: H(X_k|Y_k):
    NonZeroIndex = measUpdateProb > 0; % to prevent from log(0)
    Hafter(iPlan) = -sum(measUpdateProb(NonZeroIndex).*log(measUpdateProb(NonZeroIndex)));
    %-----------------------

    
    %-- Checking -----------
    if sim.flagDisp.after == 1
        figure(iClock+10),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
        plot(planner.param.RefPt,measUpdateProb,'b-','LineWidth',2); hold on;
        plot(planner.param.RefPt,likelihoodProb,'g--','LineWidth',2);
        plot(planner.pt,zeros(1,planner.nPt),'m.','LineWidth',3);
    end
    
    if iPlan == 1
        title('P(x_t|y_{k+1:t})','fontsize',10);
        legend('Particle-PDF','Likelihood, P(y_t|x_t)','Particle');
    end
    %----------------------
    
    
    % resample particle
    for iPt = 1:planner.nPt
        planner.pt(iPt) = planner.pt(find(rand <= cumsum(planner.w),1));
    end
    planner.w = (1/planner.nPt)*ones(1,planner.nPt);
    planner.xhat = sum(planner.w.*planner.pt')/sum(planner.w);
    
    % Mutual Information computation and accumulation for getting cost
    I = I + (Hbefore(iPlan) - Hafter(iPlan));
    %--------------
    
    
end


end