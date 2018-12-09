%-----------------------------------
% PF-based Mutual information Computation
%-----------------------------------
function [Hbefore,Hafter,I] = ComputeInformation(planner,agent,field,clock,sim,iAction,iClock)

nAgent = length(agent);
AgentID = planner.id;

I = 0;
Hbefore = nan(clock.nT,1);
Hafter = nan(clock.nT,1);

element = 'agent';

for iPlan = 1:clock.nT
    
    % take initialization with respect to agent
    if iPlan > 1
        planner.param = rmfield(planner.param,element);
    end
    
    % Sum of prob. target evolution P(X_k|y_{k-1})
    targetUpdatePdf = ComputePDFMixture(planner.pt,planner.w,planner.param,'Gaussian');
    
    % Entropy computation: H(X_k|y_{k-1})
    Hbefore(iPlan,1) = ComputeEntropy(targetUpdatePdf,planner.param.pdf.dRefPt,'moon');
    
    % agent moving along with planner action
    agent(AgentID).s = UpdateAgentState(agent(AgentID).s,planner.actionSet(iPlan,iAction),clock.dt);
    
    % particle evolution using target dynamics
    planner.pt = UpdateParticle(planner.pt,planner.param,clock.dt);
    
    % P(y_k|y_{k-1})
    for iAgent = 1:nAgent
        planner.param.agent.s = agent(iAgent).s; % ad-hoc approach to address the parameters in ComputePDFMixture in sensor model
        measConditionPdf = ComputePDFMixture(planner.pt,planner.w,planner.param,'Binary');
        
        % sample measurement: for weight update of PDF approach and
        % likelihood function of Ryan's approach
        % with respect single agent
        planner.y(iAgent) = SampleMeasurement(measConditionPdf,planner.nState,planner.param.pdf.dRefPt);
    end
    
    % weight update: w_{k-1} -> w_k
    % gather agent state info for overall weight computation
    for iAgent = 1:nAgent
        planner.param.agent(iAgent).s = agent(iAgent).s;
    end
    planner.w = UpdateParticleWeight(planner.y,planner.pt,planner.param.agent,planner.param.sensor);
    
    % probability of measurement update P(X_k|y_k):
    [likelihoodPdf,measUpdatePdf] = ComputeMeasUpdatePDF(targetUpdatePdf,planner.y,planner.param.agent,planner.param.sensor,planner.param.pdf,planner.nState);

    % Entropy computation: H(X_k|Y_k):
    Hafter(iPlan,1) = ComputeEntropy(measUpdatePdf,planner.param.pdf.dRefPt,'moon');
    
    
    %-- Plot:: before resampling -----------
    % Plot P(X_k|y_{k-1}) if needed
    if sim.flagDisp.before == 1
        figure(iClock+10),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
        PlotPDF(targetUpdatePdf,planner.pt,planner.param.pdf);        
        if iPlan == 1
            title('P(x_t|y_{k+1:t-1})','fontsize',10);
        end
        ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
    end
    
    % Plot P(X_k|y_k) if needed
    if sim.flagDisp.after == 1
        figure(iClock+10),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
        PlotPDF(measUpdatePdf,planner.pt,planner.param.pdf);                
        if iPlan == 1
            title('P(x_t|y_{k+1:t})','fontsize',10);
            legend('PDF','Particle');
        end
    end
    %----------------------
    
    % resample particle
    [planner.pt,planner.w] = ResampleParticle(planner.pt,planner.w,field);
    
    % take centroid of particles
    planner.x = (planner.w*planner.pt')';
    
    % Mutual Information computation and accumulation for getting cost
    I = I + (Hbefore(iPlan) - Hafter(iPlan));
    %--------------
    
    
end


end