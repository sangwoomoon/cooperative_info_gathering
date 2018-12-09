%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% measurements
%-----------------------------------
function [Hbefore,Hafter,I] = ComputeInformationMeasConsider(planner,agent,field,clock,bPdfDisp,iAction,iClock,id)

nMeasSet = length(planner.measSet(1,:));
nCommSet = length(planner.commSet(1,:));

I = 0;
Hbefore = zeros(clock.nT,1);
Hafter = zeros(clock.nT,1);


for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement,Ielement] = ...
            ComputeInformationSinleMeasCommSet(planner,agent,field,clock,bPdfDisp,iAction,iClock,iMeas,iComm,id);
        
        I = I + sum(Ielement);
        Hbefore = Hbefore + sum(HbeforeElement,1)';
        Hafter = Hafter + sum(HafterElement,1)';
        
    end
     
end

% divide by number of all possible events: in order to fit the equation
nPossibleEvents = nMeasSet*nCommSet;
I = I/nPossibleEvents;
Hbefore = Hbefore/nPossibleEvents;
Hafter = Hafter/nPossibleEvents;

end


function [Hbefore,Hafter,I] = ComputeInformationSinleMeasCommSet(planner,agent,field,clock,bPdfDisp,iAction,iClock,iMeas,iComm,id)

nAgent = length(agent);
nTarget = length(planner.PTset);


I = zeros(nTarget,1);
Hbefore = nan(nTarget,clock.nT);
Hafter = nan(nTarget,clock.nT);


for iPlan = 1:clock.nT
    
    for iTarget = 1:nTarget
        % take measurement from measurement set
        planner.y = planner.measSet(iPlan,iMeas);
    
        % take communication delivery from communication set
        % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
        planner.z = planner.commSet(iPlan,iComm);
    
        % Sum of prob. target evolution P(X_k|Z_{k-1})
        targetUpdatePdf = ComputePDFMixture(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,planner.param,'Gaussian');
    
        % Entropy computation: H(X_k|Z_{k-1})
        Hbefore(iTarget,iPlan) = ComputeEntropy(targetUpdatePdf,planner.param.pdf.dRefPt,'moon');
    
        % agent moving along with planner action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        agent(planner.id).s = UpdateAgentState(agent(planner.id).s,planner.actionSet(iPlan,iAction),clock.dt);
        
        % particle evolution using target dynamics
        planner.pt = UpdateParticle(planner.PTset(iTarget).pt,planner.param,clock.dt);
    
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        for iAgent = 1:nAgent
            planner.param.agent(iAgent).s = agent(iAgent).s;
        end
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,planner.param.agent,planner.param.sensor);
    
        % probability of measurement update P(X_k|Z_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [~,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,agent,planner.y,planner.z,planner.param.agent,planner.param.sensor,planner.param.pdf,planner.PTset(iTarget).nState,id);
    
        % Entropy computation: H(X_k|Z_k):
        Hafter(iTarget,iPlan) = ComputeEntropy(measUpdatePdf,planner.param.pdf.dRefPt,'moon');
    
    
        %-- Plot:: before resampling -----------
        % Plot P(X_k|y_{k-1}) if needed
        if bPdfDisp.before == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotPDF(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf);
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
        end
        
        % Plot P(X_k|y_k) if needed
        if bPdfDisp.after == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotPDF(measUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf);
            ylabel(['y_t =',num2str(planner.y)],'fontsize',12);
            if iPlan == 1
                title('P(x_t|y_{k+1:t})','fontsize',10);
                legend('PDF','Particle');
            end
        end
        %----------------------
    
        % resample particle
        [planner.PTset(iTarget).pt,planner.PTset(iTarget).w] = ...
            ResampleParticle(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,field);
    
        % take weighted mean of particles
        planner.xSet(iTarget).x = (planner.PTset(iTarget).w*planner.PTset(iTarget).pt')';
    
        % Mutual Information computation and accumulation for getting cost
        I(iTarget) = I(iTarget) + (Hbefore(iTarget,iPlan) - Hafter(iTarget,iPlan));
        %--------------
    
    end
    
end


end