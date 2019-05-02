%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% communication scenario
%
% FIVE APPROACHES are implemented for analysis
%
% 1. pmAll:       particle method considers all measurement/communication awareness possibilities.
% 2. pmSample:    particle method of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
% 3. gaussSample: Gaussian approximation of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
% 4. gaussRtilde: Gaussian approximation with modified covariance approach: Maicej's approach
% 5. gaussAll:    Gaussian with all measurement/communication possibilities: exact when the model is Linear/Gaussian
%
% Agent 1 receives information from Agent 2
%-----------------------------------
function [pmAll, pmSample, pmSeparate, gaussRtilde, gaussAll] = ComputeInformation(iAgent,iAction,iClock,sim)

planner = sim.planner(iAgent);
plannerClock = planner.param.clock;

bPdfDisp = sim.flagDisp;
flagComm = sim.flagComm;
flagPdfCompute = sim.flagPdfCompute;
flagSensor = sim.flagSensor;

nMeasSet = length(planner.measSet(1,:));
nCommSet = length(planner.commSet(1,:));

% mutual information from particle method
pmAll.Hbefore = zeros(plannerClock.nT,1);
pmAll.Hafter = zeros(plannerClock.nT,1);
pmAll.I = nan(plannerClock.nT,1);

% mutual information from particle method, which considers communication
% probability and mutual information separatively
pmSeparate.Hbefore = zeros(plannerClock.nT,1);
pmSeparate.Hafter = zeros(plannerClock.nT,1);
pmSeparate.I = nan(plannerClock.nT,1);

% mutual information under Gaussian assumption: exact solution since it
% considers all communication-aware events
gaussAll.Hbefore = zeros(plannerClock.nT,1);
gaussAll.Hafter = zeros(plannerClock.nT,1);
gaussAll.I = nan(plannerClock.nT,1);

if flagComm == 1
    
    % initialization of mutual info by Gaussian assumption with
    % modified sensor noise covariance matrix: what Maicej did
    gaussRtilde.I = nan(plannerClock.nT,1);
    
    % initialization of mutual info by Particle Method by sampling events
    % concept of Ryan's approach
    pmSample.I = nan(plannerClock.nT,1);
    
end
%---


%---------------------------------------------------------------------
% APPROACH #4: Sangwoo PM
%
% compute information under the all possibilities: particle-method
% MI = ?[P(MI_i)*MI_i] : weighted sum of elements of MI with respect to probability of the element of communication tree

tic;

for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement,commProb] = ...
            ComputeInformationByParticleMethod(iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp);
        
        % entropy and mutual information from particle method
        pmAll.Hbefore = pmAll.Hbefore + prod(commProb)*sum(HbeforeElement,2);
        pmAll.Hafter = pmAll.Hafter + prod(commProb)*sum(HafterElement,2);      
        %---
        
    end
     
end

pmAll.time = toc;
%---------------------------------------------------------------------


%---------------------------------------------------------------------
% APPROACH #5: Exact Solution under Linear-Gaussian
%
% compute information under the all possibilities: exact information computation under linear-gaussian model
% MI = ?[P(MI_i)*MI_i] : weighted sum of elements of MI with respect to probability of the element of communication tree
for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement,commProb] = ...
            ComputeInformationByGaussianCommAware(iMeas,iComm,iClock,iAction,planner,bPdfDisp,flagSensor,flagComm);
        
        % entropy and mutual information under Gaussian Assumption
        gaussAll.Hbefore = gaussAll.Hbefore + prod(commProb)*sum(HbeforeElement,2);
        gaussAll.Hafter = gaussAll.Hafter + prod(commProb)*sum(HafterElement,2);        
        %---
        
    end
     
end
%---------------------------------------------------------------------



if flagComm == 1
    
    
    %---------------------------------------------------------------------
    % APPROACH #1: Sangwoo PM by computing MI = ?[?*I(X;Y)]
    %
    % compute information under the all possibilities: particle-method that
    % considers measurement and communication separately
    % MI = ?[P(MI_i(X;Y))*MI_i(X;Y)] : weighted sum of elements of MI with respect to probability of the element of communication tree
    for iMeas = 1:nMeasSet
        
        for iComm = 1:nCommSet
            
            [HbeforeElement,HafterElement,commProb] = ...
                ComputeInformationByParticleMethodSeparateComm(iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp);
            
            % entropy and mutual information from particle method
            pmSeparate.Hbefore = pmSeparate.Hbefore + prod(commProb)*sum(HbeforeElement,2);
            pmSeparate.Hafter = pmSeparate.Hafter + prod(commProb)*sum(HafterElement,2);
            %---
            
        end
        
    end
    %---------------------------------------------------------------------

    
    %---------------------------------------------------------------------
    % APPROACH #3: modified R matrix by Majcej's expected measurement approximation
    %
    % compute information using
    % Linear-Gaussian Assumption (KF concept): Maicej's work, it does not
    % consider all possibilities of communication event
    %
    % mutual information under Gaussian Assumption
    % Refer "M.Stachura & E.Frew, Communication-Aware
    %  Information-Gathering Experiments with an Unmanned Aircraft System"
    %  to compute communication-aware Entropy
    [gaussRtilde.Hbefore,gaussRtilde.Hafter] = ...
        ComputeInformationByCovMatrixApproximation(iClock,iAction,planner,flagSensor,flagComm,bPdfDisp);
    
    gaussRtilde.Hbefore = sum(gaussRtilde.Hbefore,2);
    gaussRtilde.Hafter = sum(gaussRtilde.Hafter,2);
    %---------------------------------------------------------------------


    %---------------------------------------------------------------------
    % APPROACH #2: PM with Sampled Communication Output MI = I(X;Z_sample)
    %
    % compute information using Paritcle filter-based approach using sampled
    % communcation output prediction, its concept is from what Ryan did
    %
    % Refer "A. Ryan & J. Hedrick, Particle filter based information-theoretic
    % active sensing"
    
    tic;
    
    % initialization
    pmSample.Hbefore = 0;
    pmSample.Hafter = 0;
    nSample = planner.param.nSample; % MC-based sampled approach
    
    % take nSample smpling procedure 
    for iSample = 1 : nSample
        [Hbefore,Hafter] = ...
            ComputeInformationByParticleMethodSampledCommOutput(iMeas,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp);
        pmSample.Hbefore = pmSample.Hbefore + Hbefore;
        pmSample.Hafter = pmSample.Hafter + Hafter;
    end
    
    % take average procedure wrt nSample
    pmSample.Hbefore = pmSample.Hbefore/nSample;
    pmSample.Hafter = pmSample.Hafter/nSample;
    
    pmSample.Hbefore = sum(pmSample.Hbefore,2);
    pmSample.Hafter = sum(pmSample.Hafter,2);
    
    pmSample.time = toc;
    %---------------------------------------------------------------------

    
end

% compute cost function, which is I(X_{k:k+t};Z_{k:k+t})
for iClock = 1:plannerClock.nT
    pmAll.I(iClock) = sum(pmAll.Hbefore(1:iClock) - pmAll.Hafter(1:iClock));
    gaussAll.I(iClock) = sum(gaussAll.Hbefore(1:iClock) - gaussAll.Hafter(1:iClock));
    if flagComm == 1
        pmSeparate.I(iClock) = sum(pmSeparate.Hbefore(1:iClock) - pmSeparate.Hafter(1:iClock));
        pmSample.I(iClock) = sum(pmSample.Hbefore(1:iClock) - pmSample.Hafter(1:iClock));
        gaussRtilde.I(iClock) = sum(gaussRtilde.Hbefore(1:iClock) - gaussRtilde.Hafter(1:iClock));
    end
end
%---

% make entropy history
HbeforeIdx = 1:2:2*plannerClock.nT-1;
HafterIdx = 2:2:2*plannerClock.nT;

pmAll.hist.H(HbeforeIdx) = pmAll.Hbefore;
pmAll.hist.H(HafterIdx) = pmAll.Hafter;

gaussAll.hist.H(HbeforeIdx) = gaussAll.Hbefore;
gaussAll.hist.H(HafterIdx) = gaussAll.Hafter;

if flagComm == 1
    
    pmSample.hist.H(HbeforeIdx) = pmSample.Hbefore;
    pmSample.hist.H(HafterIdx) = pmSample.Hafter;
    
    gaussRtilde.hist.H(HbeforeIdx) = gaussRtilde.Hbefore;
    gaussRtilde.hist.H(HafterIdx) = gaussRtilde.Hafter;
    
    pmSeparate.hist.H(HbeforeIdx) = pmSeparate.Hbefore;
    pmSeparate.hist.H(HafterIdx) = pmSeparate.Hafter;
    
end

% make time history with respect to entropy change
tIdx = 1:plannerClock.nT;

pmAll.hist.time(HbeforeIdx) = tIdx;
pmAll.hist.time(HafterIdx) = tIdx;

gaussAll.hist.time(HbeforeIdx) = tIdx;
gaussAll.hist.time(HafterIdx) = tIdx;

if flagComm == 1
    
    pmSample.hist.time(HbeforeIdx) = tIdx;
    pmSample.hist.time(HafterIdx) = tIdx;
    
    gaussRtilde.hist.time(HbeforeIdx) = tIdx;
    gaussRtilde.hist.time(HafterIdx) = tIdx;

    pmSeparate.hist.time(HbeforeIdx) = tIdx;
    pmSeparate.hist.time(HafterIdx) = tIdx;
    
else
    pmSample = [];
    gaussRtilde = [];
    pmSeparate = [];
end

end


% Particle Method
function [Hbefore,Hafter,commProb] = ...
    ComputeInformationByParticleMethod(iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp)


plannerAgent = planner.param.agent;
plannerSensor = planner.param.sensor;
plannerField = planner.param.field;
plannerClock = planner.param.clock;
plannerTarget = planner.param.target;

nAgent = length(plannerAgent);
nTarget = length(planner.PTset);

% initialization of entropy and mutual info by particle method
Hbefore = nan(plannerClock.nT,nTarget);
Hafter = nan(plannerClock.nT,nTarget);

% Communication probability initialization
commProb = ones(1,nAgent);

for iPlan = 1:plannerClock.nT
    
    % predicted agent state by given planner's action:
    % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
    plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
    
    for iTarget = 1:nTarget
        
        % initialize measurement set as null group
        planner.y = nan(length(plannerSensor.R(:,1)),nAgent);
        
        % predicted target state by state update to take virtual measurement
        plannerTarget(iTarget).x = UpdateTargetState(plannerTarget(iTarget).x,planner.param,plannerClock.dt);
        
        % Sum of prob. target evolution P(X_k|Z_{k-1})
        % in order to improve the computation for computing entropy from
        % the approximated PDF, cylinder approach is developed and can be
        % compared with discretized domain
        targetUpdatePdf = ComputePDFMixture(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,planner.param,flagPdfCompute);
        
        % Entropy computation: H(X_k|Z_{k-1})
        Hbefore(iPlan,iTarget) = ComputeEntropy(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        % particle evolution using target dynamics
        planner.PTset(iTarget).pt = UpdateParticle(planner.PTset(iTarget).pt,planner.param,plannerClock.dt);
        
        % take communication delivery from communication set
        % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
        ptAgent = 1;
        for iAgent = 1:nAgent
            if iAgent == planner.id
                planner.z(iAgent) = 1;
            else
                planner.z(iAgent) = planner.commSet((nAgent-1)*(iPlan-1)+ptAgent,iComm);
                ptAgent = ptAgent + 1;
            end
        end
        
        % take measurement from measurement set
        switch flagSensor
            case 'detection'
                % take measurement from measurement set
                planner.y = planner.measSet(iPlan,iMeas);
            otherwise
                % for other measurements: uses target or agent states with
                % parameters given by sensor/planner class
                
                % the planner considers all predicted measurements with
                % respect to all agents, of which other agents' states are delivered from
                % other agents/predicted by its own agent
                %
                % now, agent 1's delivered measurement from agent 2 is determined by
                % communication set
                for iAgent = 2: nAgent
                    if planner.z(iAgent)
                        planner.y(:,iAgent) = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                    else
                        planner.y(:,iAgent) = nan(length(plannerSensor.R(:,1)),1);
                    end
                end
                
        end

        
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,...
            plannerAgent,plannerSensor,flagSensor);
        
        % probability of measurement update P(X_k|Z_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [commProbSinglePlan,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,planner.PTset(iTarget).pt,plannerAgent,planner.param,planner.y,planner.z,planner.id,flagComm,flagSensor,flagPdfCompute);
        
        % Entropy computation: H(X_k|Z_k):
        Hafter(iPlan,iTarget) = ComputeEntropy(measUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        
        %-- Plot:: before resampling -----------
        % Plot P(X_k|y_{k-1}) if needed
        if bPdfDisp.before == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotPDF(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
            axis([plannerField.boundary]);
        end
        
        % resample particle if connected
        % leave particle set if disconnected
        if planner.z(2)
            [planner.PTset(iTarget).pt,planner.PTset(iTarget).w] = ...
                ResampleParticle(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,plannerField);
        end
        
        
        % Plot P(X_k|y_k) if needed
        if bPdfDisp.after == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotPDF(measUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            
            if iPlan == 1
                title('P(x_t|y_{k+1:t})','fontsize',10);
                % legend('PDF','Particle');
            end
            axis([plannerField.boundary]);
        end
        %----------------------
        
    end
    
    commProb = commProb.*commProbSinglePlan;
    
end


end


% Particle Method by taking measurement and communication model
% separatively
function [Hbefore,Hafter,commProb] = ...
    ComputeInformationByParticleMethodSeparateComm(iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp)


plannerAgent = planner.param.agent;
plannerSensor = planner.param.sensor;
plannerField = planner.param.field;
plannerClock = planner.param.clock;
plannerTarget = planner.param.target;

nAgent = length(plannerAgent);
nTarget = length(planner.PTset);

% initialization of entropy and mutual info by particle method
Hbefore = nan(plannerClock.nT,nTarget);
Hafter = nan(plannerClock.nT,nTarget);

% Communication probability initialization
commProb = ones(1,nAgent);

for iPlan = 1:plannerClock.nT
    
    % predicted agent state by given planner's action:
    % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
    plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
    
    for iTarget = 1:nTarget
        
        % initialize measurement set as null group
        planner.y = nan(length(plannerSensor.R(:,1)),nAgent);
        
        % predicted target state by state update to take virtual measurement
        plannerTarget(iTarget).x = UpdateTargetState(plannerTarget(iTarget).x,planner.param,plannerClock.dt);
        
        
        % Sum of prob. target evolution P(X_k|Y_{k-1})
        % in order to improve the computation for computing entropy from
        % the approximated PDF, cylinder approach is developed and can be
        % compared with discretized domain
        targetUpdatePdf = ComputePDFMixture(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,planner.param,flagPdfCompute);
        
        % Entropy computation: H(X_k|Y_{k-1})
        Hbefore(iPlan,iTarget) = ComputeEntropy(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        % particle evolution using target dynamics
        planner.PTset(iTarget).pt = UpdateParticle(planner.PTset(iTarget).pt,planner.param,plannerClock.dt);
        
        % allocate communication probability
        ptAgent = 1;
        for iAgent = 1:nAgent
            if iAgent == planner.id
                planner.z(iAgent) = 1;
            else
                planner.z(iAgent) = planner.commSet((nAgent-1)*(iPlan-1)+ptAgent,iComm);
                ptAgent = ptAgent + 1;
            end
        end
        
        % take measurement from measurement set
        switch flagSensor
            case 'detection'
                % take measurement from measurement set
                planner.y = planner.measSet(iPlan,iMeas);
            otherwise
                % for other measurements: uses target or agent states with
                % parameters given by sensor/planner class
                
                % the planner considers all predicted measurements with
                % respect to all agents, of which other agents' states are delivered from
                % other agents/predicted by its own agent
                %
                % now, agent 1's delivered measurement from agent 2 is determined by
                % communication set
                %
                % since the mutual information does not consider
                % communication (I = I(X;Y)), measurements are always
                % taken regardless of Z.
                for iAgent = 2: nAgent
                    planner.y(:,iAgent) = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                end
                
        end

        
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,...
            plannerAgent,plannerSensor,flagSensor);
        
        % probability of measurement update P(X_k|Y_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [commProbSinglePlan,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,planner.PTset(iTarget).pt,plannerAgent,planner.param,planner.y,planner.z,planner.id,flagComm,flagSensor,flagPdfCompute);
        
        % Entropy computation: H(X_k|Z_k):
        Hafter(iPlan,iTarget) = ComputeEntropy(measUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        
        %-- Plot:: before resampling -----------
        % Plot P(X_k|y_{k-1}) if needed
        if bPdfDisp.before == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotPDF(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
            axis([plannerField.boundary]);
        end
        
        % resample particle
        [planner.PTset(iTarget).pt,planner.PTset(iTarget).w] = ...
            ResampleParticle(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,plannerField);
        
        
        % Plot P(X_k|y_k) if needed
        if bPdfDisp.after == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotPDF(measUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            
            if iPlan == 1
                title('P(x_t|y_{k+1:t})','fontsize',10);
                % legend('PDF','Particle');
            end
            axis([plannerField.boundary]);
        end
        %----------------------
        
    end
    
    commProb = commProb.*commProbSinglePlan;
    
end


end


% Sampled Concept of Ryan's approach: sample communication output
% z_sampled ~ P_co(z|y)
function [Hbefore,Hafter] = ...
    ComputeInformationByParticleMethodSampledCommOutput(iMeas,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp)


plannerAgent = planner.param.agent;
plannerSensor = planner.param.sensor;
plannerField = planner.param.field;
plannerClock = planner.param.clock;
plannerTarget = planner.param.target;

nAgent = length(plannerAgent);
nTarget = length(planner.PTset);

% initialization of entropy and mutual info by particle method
Hbefore = nan(plannerClock.nT,nTarget);
Hafter = nan(plannerClock.nT,nTarget);

% initialization of beta probability
beta = nan(1,nAgent);

for iPlan = 1:plannerClock.nT
    
    % predicted agent state by given planner's action:
    % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
    plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
    
    for iTarget = 1:nTarget
        
        % initialize measurement set as null group
        planner.y = nan(length(plannerSensor.R(:,1)),nAgent);
        
        % predicted target state by state update to take virtual measurement
        plannerTarget(iTarget).x = UpdateTargetState(plannerTarget(iTarget).x,planner.param,plannerClock.dt);
        
        
        % Sum of prob. target evolution P(X_k|Z_{k-1})
        % in order to improve the computation for computing entropy from
        % the approximated PDF, cylinder approach is developed and can be
        % compared with discretized domain
        targetUpdatePdf = ComputePDFMixture(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,planner.param,flagPdfCompute);
        
        % Entropy computation: H(X_k|Z_{k-1})
        Hbefore(iPlan,iTarget) = ComputeEntropy(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        % particle evolution using target dynamics
        planner.PTset(iTarget).pt = UpdateParticle(planner.PTset(iTarget).pt,planner.param,plannerClock.dt);
        
        % sample z based on P_co(z|y)
        for iAgent = 1:nAgent
            beta(iAgent) = ComputeCommProb(plannerAgent(planner.id).s,plannerAgent(iAgent).s);
            planner.z(iAgent) = binornd(1,beta(iAgent));
        end

        % take measurement from measurement set
        switch flagSensor
            case 'detection'
                % take measurement from measurement set
                planner.y = planner.measSet(iPlan,iMeas);
            otherwise
                % for other measurements: uses target or agent states with
                % parameters given by sensor/planner class
                
                % the planner considers all predicted measurements with
                % respect to all agents, of which other agents' states are delivered from
                % other agents/predicted by its own agent
                %
                % now, agent 1's delivered measurement from agent 2 is determined by
                % communication set
                for iAgent = 2: nAgent
                    if planner.z(iAgent)
                        planner.y(:,iAgent) = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                    else
                        planner.y(:,iAgent) = nan(length(plannerSensor.R(:,1)),1);
                    end
                end
                
        end
                
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,...
            plannerAgent,plannerSensor,flagSensor);
        
        % probability of measurement update P(X_k|Z_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [~,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,planner.PTset(iTarget).pt,plannerAgent,planner.param,planner.y,planner.z,planner.id,flagComm,flagSensor,flagPdfCompute);
        
        % Entropy computation: H(X_k|Z_k):
        Hafter(iPlan,iTarget) = ComputeEntropy(measUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);
        
        
        %-- Plot:: before resampling -----------
        % Plot P(X_k|y_{k-1}) if needed
        if bPdfDisp.before == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotPDF(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            if iPlan == 1
                title('P(x_t|y_{k+1:t-1})','fontsize',10);
            end
            ylabel(['t =',num2str(iPlan+iClock)],'fontsize',12);
            axis([plannerField.boundary]);
        end
        
        % resample particle when connected
        % leave particle set when disconnected
        if planner.z(2)
            [planner.PTset(iTarget).pt,planner.PTset(iTarget).w] = ...
                ResampleParticle(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,plannerField);
        end
        
        
        % Plot P(X_k|y_k) if needed
        if bPdfDisp.after == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotPDF(measUpdatePdf,planner.PTset(iTarget).pt,planner.param.pdf,flagPdfCompute);
            
            if iPlan == 1
                title('P(x_t|y_{k+1:t})','fontsize',10);
                % legend('PDF','Particle');
            end
            axis([plannerField.boundary]);
        end
        %----------------------
        
    end
    
end


end


% Maicej's approach: modify measurement covariance matrix by beta
function [Hbefore,Hafter] = ...
    ComputeInformationByCovMatrixApproximation(iClock,iAction,planner,flagSensor,flagComm,bPdfDisp)

plannerClock = planner.param.clock;

F = planner.param.F;
Q = planner.param.Q;

nAgent = length(planner.param.agent);
nTarget = length(planner.param.target);
nPlan = plannerClock.nT;

plannerField = planner.param.field;

HbeforeElement = nan(nPlan,nTarget);
HafterElement = nan(nPlan,nTarget);


for iTarget = 1:nTarget

    R = planner.param.sensor.R;
    
    xhat = planner.PTset(iTarget).xhat;
    Phat = planner.PTset(iTarget).Phat;
    
    nState = length(xhat);
    
    plannerAgent = planner.param.agent;
    
    for iPlan = 1:nPlan
            
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
        
        % state update
        xhat = F*xhat;
        Phat = F*Phat*F' + Q;
        
        % plotting distribution: before taking measurement prediction
        if bPdfDisp.before == 1
            figure(iClock+10+1+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotGaussianEllipsoid(xhat,Phat);
            axis([plannerField.boundary]);
        end
        
        % compute entropy before taking measurement
        HbeforeElement(iPlan,iTarget) = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));
        
        
        % take mearurement prediction based on the estimator and update
        for iAgent = 2: nAgent
            % when the planner consider communication, then the mofified noise
            % covariance (from M.Stachura & E.Frew, Communication-Aware
            % Information-Gathering Experiments with an Unmanned Aircraft System)
            % is computed
            if flagComm == 1
                beta = ComputeCommProb(plannerAgent(planner.id).s,plannerAgent(iAgent).s);
                paramSensor.R = planner.param.sensor.R/beta;
                R = paramSensor.R;
                % sample measurement prediction based on modified R
                planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,paramSensor,flagSensor);
            else
                % sample measurement prediction based on planner's R
                planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);                
            end
                   
            % measurement update

            % take linearized measurement matrix
            H = ComputeMeasurementMatrix(xhat,plannerAgent(iAgent).s,flagSensor);
            % make zero noise cov matrix to get measurement residual
            idealSensorParam.R = zeros(size(planner.param.sensor.R));
            K = Phat*H'*(R+H*Phat*H')^(-1);
            xhat = xhat + K*(planner.y(:,iAgent) - TakeMeasurement(xhat,plannerAgent(iAgent).s,idealSensorParam,flagSensor));
            switch flagSensor
                case 'PosLinear'
                    Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';
                case 'range_bear'
                    Phat = (eye(nState)-K*H)*Phat;
            end
        end
        
        % plotting distribution: after
        if bPdfDisp.after == 1
            figure(iClock+10+1+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotGaussianEllipsoid(xhat,Phat);
            axis([plannerField.boundary]);
        end
        
        % compute entropy after taking measurement
        HafterElement(iPlan,iTarget) = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));
        % Hafter = 1/2*log((2*pi*exp(1))^nState*det(Phat));
        
    end
    
end

Hbefore = sum(HbeforeElement,2);
Hafter = sum(HafterElement,2);

end


% exact approach: consider all posssible events for communication-awareness
% under linear-Gaussian distribution
function [Hbefore,Hafter,commProb] = ...
    ComputeInformationByGaussianCommAware(iMeas,iComm,iClock,iAction,planner,bPdfDisp,flagSensor,flagComm)

plannerClock = planner.param.clock;

nAgent = length(planner.param.agent);
nTarget = length(planner.PTset);

F = planner.param.F;
Q = planner.param.Q;

% initialization of entropy and mutual info by Gaussian assumption with all
% possible communication-aware events: exact solution
Hbefore = nan(plannerClock.nT,nTarget);
Hafter = nan(plannerClock.nT,nTarget);

% overall communication probability initialization
commProb = ones(1,nAgent);

for iTarget = 1:nTarget
    
    R = planner.param.sensor.R;
    
    xhat = planner.PTset(iTarget).xhat;
    Phat = planner.PTset(iTarget).Phat;
    
    nState = length(xhat);
    
    plannerAgent = planner.param.agent;
    plannerField = planner.param.field;
    
    for iPlan = 1:plannerClock.nT
        
        % communication probability for single planning step initialization
        commProbSinglePlan = ones(1,nAgent);
        % sampled measurement prediction initialization
        planner.y = nan(length(R(:,1)),nAgent);
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
                
        % state update
        xhat = F*xhat;
        Phat = F*Phat*F' + Q;
        
        % plotting distribution: before taking measurement prediction
        if bPdfDisp.before == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
            PlotGaussianEllipsoid(xhat,Phat);
            axis([plannerField.boundary]);
        end
        
        % compute entropy before taking measurement
        Hbefore(iPlan,iTarget) = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));
        
        % AGENT 1 JUST USES THE INFORMATION FROM OTHER AGENTS!
        
        % trivial case (ad-hoc implementation): set communication probability of
        % its ownship communication situation. This is because of the simulation situation that the agent 1 does not
        % take measurement itself.
        planner.z(planner.id) = 1;
        commProbSinglePlan(planner.id) = planner.z(planner.id);
        
        ptAgent = 1;
        for iAgent = 2: nAgent
            
            % take communication delivery from communication set
            % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
            if flagComm
                
                if iAgent ~= planner.id
                    planner.z(iAgent) = planner.commSet((nAgent-1)*(iPlan-1)+ptAgent,iComm);
                    ptAgent = ptAgent + 1;
                end
                
                % take measurement/communication prediction if connected
                if planner.z(iAgent)
                    planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                    commProbSinglePlan(iAgent) = ComputeCommProb(plannerAgent(planner.id).s,plannerAgent(iAgent).s);
                else
                    planner.y(:,iAgent) = nan(length(R(:,1)),1);
                    commProbSinglePlan(iAgent) = 1 - ComputeCommProb(plannerAgent(planner.id).s,plannerAgent(iAgent).s);
                end
                
            else
                planner.z(iAgent) = 1;
                planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                commProbSinglePlan(iAgent) = 1;
            end
        
            % measurement update
            
            % when the planner consider communication, it depends on given
            % communication output
            if planner.z(iAgent) == 1
                % take linearized measurement matrix
                H = ComputeMeasurementMatrix(xhat,plannerAgent(iAgent).s,flagSensor);
                % make zero noise cov matrix to get measurement residual
                idealSensorParam.R = zeros(size(planner.param.sensor.R));
                K = Phat*H'*(R+H*Phat*H')^(-1);
                xhat = xhat + K*(planner.y(:,iAgent) - TakeMeasurement(xhat,plannerAgent(iAgent).s,idealSensorParam,flagSensor));
                switch flagSensor
                    case 'PosLinear'
                        Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';
                    case 'range_bear'
                        Phat = (eye(nState)-K*H)*Phat;
                end
            end
            
        end
        
        % plotting distribution: after
        if bPdfDisp.after == 1
            figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
            PlotGaussianEllipsoid(xhat,Phat);
            axis([plannerField.boundary]);
        end
        
        % compute entropy after taking measurement
        Hafter(iPlan,iTarget) = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));
        % Hafter = 1/2*log((2*pi*exp(1))^nState*det(Phat));
        
        % computation of commProb does not have to do with respect to
        % targets.
        if iTarget == 1
           commProb = commProb.*commProbSinglePlan; 
        end
        
    end
    
end

end

