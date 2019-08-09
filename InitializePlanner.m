function planner = InitializePlanner(iAgent,sim, dt, nT, nPt, dRefPt, nSample )

D2R = pi/180;

nAgent = sim.nAgent;
nTarget = sim.nTarget;
flagComm = sim.flagComm;
flagSensor = sim.flagSensor;
flagFilter = sim.flagFilter;

agent = sim.agent;
sensor = sim.sensor;
field = sim.field;
filter = sim.filter;

planner.id = iAgent;

planner.param.clock.dt = dt; % planning time-step horizon
planner.param.clock.nT = nT; % planning horizon
planner.param.nSample = nSample; % sampled specific measurements set
%         planner(iPlanner).param.sA = 3; % sampled action

% action profile setting
[planner.action,planner.actionNum,planner.actionSetNum,planner.actionSet] = GenerateOutcomeProfile([-20,0,20]*D2R,planner.param.clock.nT);

% measurement profile setting
switch flagSensor
    case 'PosLinear'
        [planner.meas,planner.measNum,planner.measSetNum,planner.measSet] = GenerateOutcomeProfile(1,planner.param.clock.nT);
    case 'range_bear'
        [planner.meas,planner.measNum,planner.measSetNum,planner.measSet] = GenerateOutcomeProfile(1,planner.param.clock.nT);        
    case 'detection'
        [planner.meas,planner.measNum,planner.measSetNum,planner.measSet] = GenerateOutcomeProfile([0 1],planner.param.clock.nT);        
end

% communication profile setting

% commSet structure
% row: firstly take sub-group of which # of row is nAgent -> take
% element that represents specific condition wrt receding time and
% agent
if flagComm
    [planner.comm,planner.commNum,planner.commSetNum,planner.commSet] = GenerateOutcomeProfile([0 1],planner.param.clock.nT*(nAgent-1));
else
    [planner.comm,planner.commNum,planner.commSetNum,planner.commSet] = GenerateOutcomeProfile(1,planner.param.clock.nT*(nAgent-1));
end


for iTarget = 1:nTarget
    
    if strcmp(flagFilter,'PF')
        planner.PTset(iTarget).nPt = filter(iAgent,iTarget).nPt;
        planner.PTset(iTarget).pt = filter(iAgent,iTarget).pt;
        planner.PTset(iTarget).w = filter(iAgent,iTarget).w;
    else
        
        dimension = length(filter(iAgent,iTarget).xhat);
        
        planner.PTset(iTarget).nPt = nPt;
        planner.PTset(iTarget).w = ones(1,nPt)./nPt;
        
        if iAgent == 1
            for iPt = 1 : nPt
        
                switch flagSensor
                    case 'detection'
                        switch dimension
                            case 2 % 2D
                                planner.PTset(iTarget).pt(:,iPt) = ...
                                    [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2)]';
                            case 3 % 3D
                                planner.PTset(iTarget).pt(:,iPt) = ...
                                    [field.bufferZone(1)+rand()*field.zoneLength(1) field.bufferZone(3)+rand()*field.zoneLength(2) field.bufferZone(5)+rand()*field.zoneLength(3)]';
                        end
                    otherwise
                        planner.PTset(iTarget).Phat = filter(iAgent,iTarget).Phat;
                        planner.PTset(iTarget).pt(:,iPt) = mvnrnd(filter(iAgent,iTarget).xhat,planner.PTset(iTarget).Phat)';
                end
                
            end
        else
            planner.PTset(iTarget).pt = sim.planner(1).PTset(iTarget).pt; % in order to make the same initial condition
        end
        
    end
    
    planner.PTset(iTarget).xhat = filter(iAgent,iTarget).xhat;
    planner.PTset(iTarget).Phat = filter(iAgent,iTarget).Phat;
    planner.PTset(iTarget).nState = filter(iAgent,iTarget).nState;
    
end

planner.input = nan(planner.param.clock.nT,1);
planner.actIdx = nan;
planner.hist.input = planner.input;
planner.hist.actIdx = planner.actIdx;

% information database
planner.I = nan;
planner.hist.I = planner.I;
planner.hist.Hbefore = nan(planner.param.clock.nT,1);
planner.hist.Hafter = nan(planner.param.clock.nT,1);

% reference information database: Gaussian assumption
planner.Iref = nan;
planner.hist.Iref = planner.Iref;
planner.hist.HbeforeRef = nan(planner.param.clock.nT,1);
planner.hist.HafterRef = nan(planner.param.clock.nT,1);

% pdf parameter initialization
planner.param.pdf.dRefPt = dRefPt;
[planner.param.pdf.refPt(1,:,:), planner.param.pdf.refPt(2,:,:)] = ...
    meshgrid(field.boundary(1):planner.param.pdf.dRefPt:field.boundary(2),...
    field.boundary(3):planner.param.pdf.dRefPt:field.boundary(4));

% plotting layout
planner.param.plot.row = planner.param.clock.nT;
planner.param.plot.col = 2;

% NOTE: TARGETS ARE HOMOGENEOUS - ITARGET DOESN'T MATTER
planner.param.F = filter(iAgent,iTarget).param.F;
planner.param.Q = filter(iAgent,iTarget).param.Q;

switch flagSensor
    case 'PosLinear'
        planner.param.sensor.R = sensor(iAgent,iTarget).param.R;
        planner.param.sensor.H = sensor(iAgent,iTarget).param.H;
        % 
    case 'range_bear'
        planner.param.sensor.R = sensor(iAgent,iTarget).param.R;
    case 'detection'        
        planner.param.sensor.regionRadius = sensor(iAgent,iTarget).param.regionRadius;
        planner.param.sensor.detectBeta = sensor(iAgent,iTarget).param.detectBeta;
end

planner.param.field = field;

% for Monte-Carlo based analysis for information distribution
planner.Isum = 0;
planner.HbeforeSum = 0;
planner.HafterSum = 0;

% agent state is used for agent state prediction
for iAgent = 1:nAgent
    planner.param.agent(iAgent).s = agent(iAgent).s;
end

% target state is used for target state prediction
for iTarget = 1:nTarget
    planner.param.target(iTarget).x = filter(iAgent,iTarget).xhat;
end

end