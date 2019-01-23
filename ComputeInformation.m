%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% measurements
%-----------------------------------
function [HbeforePM,HafterPM,I_PM,  HbeforeTrue,HafterTrue,Itrue] = ComputeInformation(iAgent,iAction,iClock,sim)

planner = sim.planner(iAgent);
plannerAgent = planner.param.agent;
plannerClock = planner.param.clock;

bPdfDisp = sim.flagDisp;
flagComm = sim.flagComm;
flagPdfCompute = sim.flagPdfCompute;
flagSensor = sim.flagSensor;

nMeasSet = length(planner.measSet(1,:));
nCommSet = length(planner.commSet(1,:));

% mutual information from particle method
HbeforePM = zeros(plannerClock.nT,1);
HafterPM = zeros(plannerClock.nT,1);
I_PM = nan(plannerClock.nT,1);

% mutual information under Gaussian assumption: exact solution since it
% considers all communication-aware events
HbeforeTrue = zeros(plannerClock.nT,1);
HafterTrue = zeros(plannerClock.nT,1);
Itrue = nan(plannerClock.nT,1);

% initialization of mutual info by Gaussian assumption with
% modified sensor noise covariance matrix: what Maicej did
Imaicej = nan(plannerClock.nT,1);

% initialization of mutual info by Particle Method by sampling events
% concept of Ryan's approach
Iryan = nan(plannerClock.nT,1);
%---


% compute information under the all possibilities: particle-method
for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement] = ...
            ComputeInformationByParticleMethod(iAgent,iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp);
        
        % entropy and mutual information from particle method
        HbeforePM = HbeforePM + sum(HbeforeElement,2);
        HafterPM = HafterPM + sum(HafterElement,2);      
        %---
        
    end
     
end

% compute information under the all possibilities: exact information computation under linear-gaussian model
for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement] = ...
            ComputeInformationByLinearGaussianCommAware(iAgent,iMeas,iComm,iClock,iAction,planner,bPdfDisp,flagSensor);
        
        % entropy and mutual information under Gaussian Assumption
        HbeforeTrue = HbeforeTrue + sum(HbeforeElement,2);
        HafterTrue = HafterTrue + sum(HafterElement,2);        
        %---
        
    end
     
end

% compute information using
% Linear-Gaussian Assumption (KF concept): Maicej's work, it does not
% consider all possibilities of communication event
%
% mutual information under Gaussian Assumption
% Refer "M.Stachura & E.Frew, Communication-Aware
%  Information-Gathering Experiments with an Unmanned Aircraft System"
%  to compute communication-aware Entropy
[HbeforeMaicej,HafterMaicej] = ...
    ComputeInformationByLinearGaussianSensorCovMatrixApproximation(iAgent,iClock,iAction,planner,plannerAgent,plannerClock,flagSensor,flagComm,bPdfDisp);


% compute information using Paritcle filter-based approach using sampled
% communcation output prediction, its concept is from what Ryan did
%
% Refer "A. Ryan & J. Hedrick, Particle filter based information-theoretic
% active sensing"
[HbeforeRyan,HafterRyan] = ...
    ComputeInformationByParticleMethodSampledCommOutput(iAgent,iMeas,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp);


% divide by number of all possible events: in order to fit the equation
nPossibleEvents = nMeasSet*nCommSet;

HbeforePM = HbeforePM/nPossibleEvents;
HafterPM = HafterPM/nPossibleEvents;

HbeforeTrue = HbeforeTrue/nPossibleEvents;
HafterTrue = HafterTrue/nPossibleEvents;

% compute cost function, which is I(X_{k:k+t};Z_{k:k+t})
for iClock = 1:plannerClock.nT
    Imaicej(iClock) = sum(HbeforeMaicej(1:iClock) - HafterMaicej(1:iClock));
    I_PM(iClock) = sum(HbeforePM(1:iClock) - HafterPM(1:iClock));
    Iryan(iClock) = sum(HbeforeRyan(1:iClock) - HafterRyan(1:iClock));
    Itrue(iClock) = sum(HbeforeTrue(1:iClock) - HafterTrue(1:iClock));
end
%---

% check entropy
HbeforeIdx = 1:2:2*plannerClock.nT-1;
HafterIdx = 2:2:2*plannerClock.nT;

H_PM(HbeforeIdx) = HbeforePM;
H_PM(HafterIdx) = HafterPM;

Hmaicej(HbeforeIdx) = HbeforeMaicej;
Hmaicej(HafterIdx) = HafterMaicej;

Hryan(HbeforeIdx) = HbeforeRyan;
Hryan(HafterIdx) = HafterRyan;

Htrue(HbeforeIdx) = HbeforeTrue;
Htrue(HafterIdx) = HafterTrue;

tIdx = 1:plannerClock.nT;

t(HbeforeIdx) = tIdx;
t(HafterIdx) = tIdx;

% plot the result 
figure(100)
plot(t,Hmaicej,'b-','linewidth',2); hold on;
plot(t,Hryan,'m-','Linewidth',2);
plot(t,H_PM,'r-','Linewidth',2);
plot(t,Htrue,'g-','Linewidth',2);
legend('modified cov matrix','PM: sampled Comm','PM: all events','true');
xlabel('t (receding horizon time step)');
ylabel('entropy');

figure(101)
plot(Imaicej,'b-','linewidth',2); hold on;
plot(Iryan,'m-','Linewidth',2);
plot(I_PM,'r-','Linewidth',2);
plot(Itrue,'g-','Linewidth',2);
legend('modified cov matrix','PM: sampled Comm','PM: all events','true');
xlabel('t (receding horizon time step)');
ylabel('I(X_{k:k+t};Y_{k:k+t})');

end


% Particle Method
function [Hbefore,Hafter] = ...
    ComputeInformationByParticleMethod(id,iMeas,iComm,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp)


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

for iPlan = 1:plannerClock.nT
    
    for iTarget = 1:nTarget
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
        
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
                for iAgent = 1: nAgent
                    planner.y(:,iAgent) = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                end
        end
        
        % take communication delivery from communication set
        % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
        for iAgent = 1:nAgent
            planner.z(iAgent) = planner.commSet(iPlan,iComm);
        end
        
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,...
            plannerAgent,plannerSensor,flagSensor);
        
        % probability of measurement update P(X_k|Z_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [~,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,planner.PTset(iTarget).pt,plannerAgent,planner.param,planner.y,planner.z,id,flagComm,flagSensor,flagPdfCompute);
        
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
    
end


end


% Sampled Concept of Ryan's approach: sample communication output
% z_sampled ~ P_co(z|y)
function [Hbefore,Hafter] = ...
    ComputeInformationByParticleMethodSampledCommOutput(id,iMeas,iClock,iAction,planner,flagSensor,flagComm,flagPdfCompute,bPdfDisp)


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
    
    for iTarget = 1:nTarget
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
        
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
                for iAgent = 1: nAgent
                    planner.y(:,iAgent) = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
                end
        end
        
        % sample z based on P_co(z|y)
        for iAgent = 1:nAgent
            beta(iAgent) = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
            planner.z(iAgent) = binornd(1,beta(iAgent));
        end
        
        % weight update: w_{k-1} -> w_k
        % gather agent state info for overall weight computation
        planner.PTset(iTarget).w = UpdateParticleWeight(planner.y,planner.PTset(iTarget).pt,...
            plannerAgent,plannerSensor,flagSensor);
        
        % probability of measurement update P(X_k|Z_k):
        % HERE IS THE MAJOR DIFFERENCE BY CONSIDERING COMMUNICATION
        % AWARENESS
        [~,measUpdatePdf] = ...
            ComputeCommAwareMeasUpdatePDF(targetUpdatePdf,planner.PTset(iTarget).pt,plannerAgent,planner.param,planner.y,planner.z,id,flagComm,flagSensor,flagPdfCompute);
        
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
    
end


end


% Maicej's approach: modify measurement covariance matrix by beta
function [Hbefore,Hafter] = ...
    ComputeInformationByLinearGaussianSensorCovMatrixApproximation(id,iClock,iAction,planner,plannerAgent,plannerClock,flagSensor,flagComm,bPdfDisp)

F = planner.param.F;
Q = planner.param.Q;

nAgent = length(planner.param.agent);
nTarget = length(planner.param.target);
nPlan = plannerClock.nT;

plannerField = planner.param.field;

HbeforeElement = nan(nPlan,nTarget);
HafterElement = nan(nPlan,nTarget);


for iPlan = 1:nPlan
    
    for iTarget = 1:nTarget
        
        R = planner.param.sensor(iTarget).R;
        H = planner.param.sensor(iTarget).H;
        
        xhat = planner.PTset(iTarget).xhat;
        Phat = planner.PTset(iTarget).Phat;
        
        nState = length(xhat);
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
    
        % take mearurement prediction based on the estimator
        for iAgent = 1: nAgent
            planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
        end
        
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
        
        % measurement update
        for iAgent = 1:nAgent
            % when the planner consider communication, then the mofified noise
            % covariance (from M.Stachura & E.Frew, Communication-Aware
            % Information-Gathering Experiments with an Unmanned Aircraft System)
            % is computed
            if flagComm == 1
                beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
                R = planner.param.sensor.R/beta;
            end
            
            K = Phat*H'*(R+H*Phat*H')^(-1);
            xhat = xhat + K*(planner.y(:,iAgent) - H*xhat);
            Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';
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
function [Hbefore,Hafter] = ...
    ComputeInformationByLinearGaussianCommAware(id,iMeas,iComm,iClock,planner,bPdfDisp,flagSensor)

plannerAgent = planner.param.agent;
plannerClock = planner.param.clock;
plannerTarget = planner.param.target;

nTarget = length(planner.PTset);

F = planner.param.F;
Q = planner.param.Q;

% initialization of entropy and mutual info by Gaussian assumption with all
% possible communication-aware events: exact solution
Hbefore = nan(plannerClock.nT,nTarget);
Hafter = nan(plannerClock.nT,nTarget);

for iPlan = 1:plannerClock.nT
    
    for iTarget = 1:nTarget
        
        R = planner.param.sensor(iTarget).R;
        H = planner.param.sensor(iTarget).H;
        
        xhat = planner.PTset(iTarget).xhat;
        Phat = planner.PTset(iTarget).Phat;
        
        nAgent = length(planner.param.agent);
        nState = length(xhat);
        
        plannerField = planner.param.field;
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
        
        % predicted target state by state update to take virtual measurement
        plannerTarget(iTarget).x = UpdateTargetState(plannerTarget(iTarget).x,planner.param,plannerClock.dt);
        
        % take measurement prediction
        for iAgent = 1: nAgent
            planner.y(:,iAgent) = TakeMeasurement(xhat,plannerAgent(iAgent).s,planner.param.sensor,flagSensor);
        end
        
        % take communication delivery from communication set
        % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
        planner.z = planner.commSet(iPlan,iComm);
        
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
        
        % measurement update
        for iAgent = 1:nAgent
            
            % agent itself does not consider the communication condition for its
            % own measurement update
            if iAgent == id
                K = Phat*H'*(R+H*Phat*H')^(-1);
                xhat = xhat + K*(planner.y(:,iAgent) - H*xhat);
                Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';
            else
                % when the planner consider communication, it depends on given
                % communication output
                if planner.z == 1
                    K = Phat*H'*(R+H*Phat*H')^(-1);
                    xhat = xhat + K*(planner.y(:,iAgent) - H*xhat);
                    Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';
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
        
    end
    
end

end
