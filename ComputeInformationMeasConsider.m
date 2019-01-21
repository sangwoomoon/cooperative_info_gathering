%-----------------------------------
% PF-based Mutual information Computation when considering all possible
% measurements
%-----------------------------------
function [Hbefore,Hafter,I,  HbeforeRef,HafterRef,Iref] = ComputeInformationMeasConsider(iAgent,iAction,iClock,sim)

planner = sim.planner(iAgent);
plannerClock = planner.param.clock;

bPdfDisp = sim.flagDisp;
flagComm = sim.flagComm;
flagPdfCompute = sim.flagPdfCompute;
flagSensor = sim.flagSensor;

nMeasSet = length(planner.measSet(1,:));
nCommSet = length(planner.commSet(1,:));

I = 0;
Hbefore = zeros(plannerClock.nT,1);
Hafter = zeros(plannerClock.nT,1);


% mutual information under Gaussian assumption
Iref = 0; 
HbeforeRef = zeros(plannerClock.nT,1);
HafterRef = zeros(plannerClock.nT,1);
%---

for iMeas = 1:nMeasSet
    
    for iComm = 1:nCommSet
        
        [HbeforeElement,HafterElement,Ielement, HbeforeRefElement,HafterRefElement,IrefElement] = ...
            ComputeInformationSingleMeasCommSet(planner,bPdfDisp,flagComm,flagPdfCompute,iAction,iClock,iMeas,iComm,iAgent,flagSensor);
        
        % entropy and mutual information from particle method
        I = I + sum(Ielement);
        Hbefore = Hbefore + sum(HbeforeElement,1)';
        Hafter = Hafter + sum(HafterElement,1)';
        
        
        % entropy and mutual information under Gaussian Assumption
        Iref = Iref + sum(IrefElement);
        HbeforeRef = HbeforeRef + sum(HbeforeRefElement,1)';
        HafterRef = HafterRef + sum(HafterRefElement,1)';        
        %---
        
    end
     
end

% divide by number of all possible events: in order to fit the equation
nPossibleEvents = nMeasSet*nCommSet;
I = I/nPossibleEvents;
Hbefore = Hbefore/nPossibleEvents;
Hafter = Hafter/nPossibleEvents;


% mutual information under Gaussian Assumption
Iref = Iref/nPossibleEvents;
HbeforeRef = HbeforeRef/nPossibleEvents;
HafterRef = HafterRef/nPossibleEvents;
%---

end


function [Hbefore,Hafter,I, HbeforeRef,HafterRef,Iref] = ...
    ComputeInformationSingleMeasCommSet(planner,bPdfDisp,flagComm,flagPdfCompute,iAction,iClock,iMeas,iComm,id,flagSensor)

plannerAgent = planner.param.agent;
plannerSensor = planner.param.sensor;
plannerField = planner.param.field;
plannerClock = planner.param.clock;
plannerTarget = planner.param.target;

nTarget = length(planner.PTset);

% initialization of entropy and mutual info by particle method
I = nan(nTarget,1);
Hbefore = nan(nTarget,plannerClock.nT);
Hafter = nan(nTarget,plannerClock.nT);

% initialization of entropy and mutual info by Gaussian assumption
Iref = nan(nTarget,1);
HbeforeRef = nan(nTarget,plannerClock.nT);
HafterRef = nan(nTarget,plannerClock.nT);

for iPlan = 1:plannerClock.nT
    
    for iTarget = 1:nTarget
        
        % predicted agent state by given planner's action:
        % THE PLANNER ONLY MOVES ITS OWN AGENT ONLY
        plannerAgent(planner.id).s = UpdateAgentState(plannerAgent(planner.id).s,planner.actionSet(iPlan,iAction),plannerClock.dt);
        
        % predicted target state by state update to take virtual measurement
        plannerTarget(iTarget).x = UpdateTargetState(plannerTarget(iTarget).x,planner.param,plannerClock.dt);
        
        % take measurement from measurement set
        switch flagSensor
            case 'detection'
                % take measurement from measurement set
                planner.y = planner.measSet(iPlan,iMeas);
            otherwise
                % for other measurements: uses target or agent states with
                % parameters given by sensor/planner class
                planner.y = TakeMeasurement(plannerTarget(iTarget).x,plannerAgent(planner.id).s,planner.param.sensor,flagSensor);
        end
    
        % take communication delivery from communication set
        % BEWARE OF BINARY REPRESENTATION: 0-null | 1-y
        planner.z = planner.commSet(iPlan,iComm);
        
        
        % compute information of single measurement / communication using
        % Particle Method
        [Hbefore(iTarget,iPlan),Hafter(iTarget,iPlan),I(iTarget,iPlan),planner.PTset(iTarget).pt,planner.PTset(iTarget).w] = ...
            ComputeInformationByParticleMethod(id,iPlan,iTarget,iMeas,iClock,planner,plannerAgent,plannerSensor,plannerClock,plannerField,flagSensor,flagComm,flagPdfCompute,bPdfDisp);
        
        % compute information of single measurement / communication using
        % Linear-Gaussian Assumption (KF concept)        
        [HbeforeRef(iTarget,iPlan),HafterRef(iTarget,iPlan),Iref(iTarget,iPlan),planner.PTset(iTarget).xhat,planner.PTset(iTarget).Phat] = ...
            ComputeInformationByLinearGaussian(id,iPlan,iMeas,iTarget,iClock,planner,bPdfDisp);
    
    end
    
end


end


function [Hbefore,Hafter,I,pt,w] = ...
    ComputeInformationByParticleMethod(id,iPlan,iTarget,iMeas,iClock,planner,plannerAgent,plannerSensor,plannerClock,plannerField,flagSensor,flagComm,flagPdfCompute,bPdfDisp)

% Sum of prob. target evolution P(X_k|Z_{k-1})
% in order to improve the computation for computing entropy from
% the approximated PDF, cylinder approach is developed and can be
% compared with discretized domain
targetUpdatePdf = ComputePDFMixture(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,planner.param,flagPdfCompute);

% Entropy computation: H(X_k|Z_{k-1})
Hbefore = ComputeEntropy(targetUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);

% particle evolution using target dynamics
planner.PTset(iTarget).pt = UpdateParticle(planner.PTset(iTarget).pt,planner.param,plannerClock.dt);

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
Hafter = ComputeEntropy(measUpdatePdf,planner.PTset(iTarget).pt,planner.param,flagPdfCompute);


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
[pt,w] = ...
    ResampleParticle(planner.PTset(iTarget).pt,planner.PTset(iTarget).w,plannerField);


% Plot P(X_k|y_k) if needed
if bPdfDisp.after == 1
    figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
    PlotPDF(measUpdatePdf,pt,planner.param.pdf,flagPdfCompute);

    if iPlan == 1
        title('P(x_t|y_{k+1:t})','fontsize',10);
        % legend('PDF','Particle');
    end
    axis([plannerField.boundary]);
end
%----------------------


% Mutual Information computation and accumulation for getting cost
I = Hbefore - Hafter;
%--------------

end



function [Hbefore,Hafter,I,xhat,Phat] = ComputeInformationByLinearGaussian(id,iPlan,iMeas,iTarget,iClock,planner,bPdfDisp)

F = planner.param.F;
Q = planner.param.Q;

R = planner.param.sensor(iTarget).R;
H = planner.param.sensor(iTarget).H;

xhat = planner.PTset(iTarget).xhat;
Phat = planner.PTset(iTarget).Phat;
nState = length(xhat);

y = planner.y;
z = planner.z;

plannerField = planner.param.field;

% state update
xhat = F*xhat;
Phat = F*Phat*F' + Q;

% plotting distribution: before
if bPdfDisp.before == 1
    figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*(iPlan-1)+1),
    PlotGaussianEllipsoid(xhat,Phat);
    axis([plannerField.boundary]);
end

% compute entropy before taking measurement
Hbefore = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));

% measurement update
K = Phat*H'*(R+H*Phat*H')^(-1);
xhat = xhat + K*(y - H*xhat);
Phat = (eye(nState)-K*H)*Phat*(eye(nState)-K*H)' + K*R*K';

% plotting distribution: after
if bPdfDisp.after == 1
    figure(iClock+10+iMeas+iTarget),subplot(planner.param.plot.row,planner.param.plot.col,2*iPlan),
    PlotGaussianEllipsoid(xhat,Phat);
    axis([plannerField.boundary]);
end

% compute entropy after taking measurement
Hafter = nState/2 + nState/2*log(2*pi) + 1/2*log(det(Phat));
% Hafter = 1/2*log((2*pi*exp(1))^nState*det(Phat));

% mutual information computation
I = Hbefore - Hafter;

end