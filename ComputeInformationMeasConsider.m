%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% measurements
%
% ONLY AGENT 2 TAKES MEASUREMENT IN THE SCENARIO: FOR ACC2019 DRAFT
% the code is only applicable in 2 agents scenario, where agent 1 receives
% information from agent 2 which is dependent to the communication status
%-----------------------------------
function [Hbefore,Hafter,I] = ComputeInformationMeasConsider(planner,agent,field,clock,sim,sActNum,iClock,id)

nMeasSet = length(planner.measSet(1,:));
nCommSet = length(planner.commSet(1,:));

I = 0;
Hbefore = zeros(clock.nT,1);
Hafter = zeros(clock.nT,1);


for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement,Ielement] = ...
            ComputeInformationSinleMeasCommSet(planner,agent,field,clock,sim,sActNum,iClock,iMeas,iComm,id);
        
        I = I + Ielement;
        Hbefore = Hbefore + HbeforeElement;
        Hafter = Hafter + HafterElement;
        
    end
     
end

% divide by number of all possible events: in order to fit the equation
nPossibleEvents = nMeasSet*nCommSet; % ONLY FOR ACC 2019
I = I/nPossibleEvents;
Hbefore = Hbefore/nPossibleEvents;
Hafter = Hafter/nPossibleEvents;

end


function [Hbefore,Hafter,I] = ComputeInformationSinleMeasCommSet(planner,agent,field,clock,sim,sActNum,iClock,iMeas,iComm,id)

nAgent = length(agent);

I = 0;
Hbefore = nan(clock.nT,1);
Hafter = nan(clock.nT,1);


for iPlan = 1:clock.nT
    
    % take measurement from measurement set
    planner.y = planner.measSet(iPlan,iMeas);
    
    % take communication delivery from communication set
    % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
    planner.z = planner.commSet(iPlan,iComm);
    
    % Sum of prob. target evolution P(X_k|Z_{k-1})
    targetUpdatePdf = ComputePDFMixture(planner.pt,planner.w,planner.param,'Gaussian');
    
    % Entropy computation: H(X_k|Z_{k-1})
    Hbefore(iPlan,1) = ComputeEntropy(targetUpdatePdf,planner.param.pdf.dRefPt,'moon');
    
    % agent moving along with planner action:
    % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
    agent(planner.id).s = UpdateAgentState(agent(planner.id).s,planner.actionSet(iPlan,sActNum),clock.dt);
    
    
    % particle evolution using target dynamics
    planner.pt = UpdateParticle(planner.pt,planner.param,clock.dt);
    
    % weight update: w_{k-1} -> w_k
    % gather agent state info for overall weight computation
    for iAgent = 1:nAgent
        planner.param.agent(iAgent).s = agent(iAgent).s;
    end
    planner.w = UpdateParticleWeight(planner.y,planner.pt,planner.param.agent,planner.param.sensor);
    
    % probability of measurement update P(X_k|Z_k):
    % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
    % AWARENESS
    % IF SEQUENCE IS ONLY USEFUL FOR ACC2019!
    [likelihoodPdf,measUpdatePdf] = ...
        ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,agent,planner.y,planner.z,planner.param.agent,planner.param.sensor,planner.param.pdf,planner.nState,id);
    
    % Entropy computation: H(X_k|Z_k):
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