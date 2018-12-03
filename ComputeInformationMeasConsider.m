%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% measurements
%
% agent 1 solver that uses all agents' information
%-----------------------------------
function [Hbefore,Hafter,I] = ComputeInformationMeasConsider(planner,agent,field,clock,sim,sActNum,iClock)

nMeasSet = length(planner.measSet(1,:));

I = 0;
Hbefore = zeros(clock.nT,1);
Hafter = zeros(clock.nT,1);

for iMeas = 1:nMeasSet
    
    [HbeforeElement,HafterElement,Ielement] = ComputeInformationSinleMeasSet(planner,agent,field,clock,sim,sActNum,iClock,iMeas);
    
    I = I + Ielement;
    Hbefore = Hbefore + HbeforeElement;
    Hafter = Hafter + HafterElement;
    
end


end


function [Hbefore,Hafter,I] = ComputeInformationSinleMeasSet(planner,agent,field,clock,sim,sActNum,iClock,iMeas)

nAgent = length(agent);

I = 0;
Hbefore = nan(clock.nT,1);
Hafter = nan(clock.nT,1);

element = 'agent';

    for iPlan = 1:clock.nT
        
        % take measurement from measurement set
        planner.y = ones(nAgent,1)*planner.measSet(iPlan,iMeas);
        
        % take initialization with respect to agent
        if iPlan > 1
            planner.param = rmfield(planner.param,element);
        end
        
        % Sum of prob. target evolution P(X_k|y_{k-1})
        targetUpdatePdf = ComputePDFMixture(planner.pt,planner.w,planner.param,'Gaussian');
        
        % Entropy computation: H(X_k|y_{k-1})
        Hbefore(iPlan,1) = ComputeEntropy(targetUpdatePdf,planner.param.pdf.dRefPt,'moon');
        
        % agent moving along with planner action
        for iAgent = 1:sim.nAgent
            agent(iAgent).s = UpdateAgentState(agent(iAgent).s,planner.actionSet(iPlan,sActNum(iAgent)),clock.dt);
        end
        
        % particle evolution using target dynamics
        planner.pt = UpdateParticle(planner.pt,planner.param,clock.dt);
        
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
            figure(iClock+10+iMeas),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotPDF(targetUpdatePdf,planner.pt,planner.param.pdf);
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
        end
        
        % Plot P(X_k|y_k) if needed
        if sim.flagDisp.after == 1
            figure(iClock+10+iMeas),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotPDF(measUpdatePdf,planner.pt,planner.param.pdf);
            ylabel(['y_t =',num2str(planner.y)],'fontsize',12);
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