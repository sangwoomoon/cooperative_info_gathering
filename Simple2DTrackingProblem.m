
close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 3;
nTarget = 2;
nLandMark = 1;
SIM = Simulation(nAgent,nTarget,nLandMark);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 200;
FDDFt = 1.0;
CLOCK = Clock(t0,dt,nt,FDDFt);

%--- Environment Classes Setting ----
%                      Dyn.Spec    State      
landmarkParam(1,:) = {'Static';   [ 5.0; -2.5]};

ENV = Environment();

for iLandmark = 1 : SIM.nLandmark
    landmarkSpec = landmarkParam(1,1);
    ENV.LANDMARK = Landmark('landmark',landmarkSpec{:});
    ENV.LANDMARK.DYNAMICS.Initialize(cell2mat(landmarkParam(iLandmark,2)));
    ENV.LANDMARK.DYNAMICS.SetParameters([],[],[]); % parameter setting order : bKFx, Q, RelTol, AbsTol (last two is for ODE45)
end

%--- Target Classes Setting ----
%                   Dyn.Spec    State                   Q               RelTol  AbsTol  
targetParam(1,:) = {'Dubins';   [1.0;-0.5;1.0];        [1.2;1.2;0.04];  1e-4;   1e-6};
targetParam(2,:) = {'Dubins';   [-10.0;0.1;0.0];       [1.2;1.2;0.05];  1e-4;   1e-6};


%--- Agent Classes Setting ----
%                   Dyn.Spec    State                     Q                     RelTol  AbsTol    Meas.Spec       Meas.Obj  bias             R                   Esti.Spec
agentParam(1,:) = {'Linear';    [-5.5; 0.0;  5.0; 0.0];   [0.2; 0.2];           1e-4;   1e-6;    'RelCartBias';  'Target';  [ 0.2; -0.3];    [10.15; 0.015];    'KF'};
agentParam(2,:) = {'Linear';    [ 1.5; 0.0; -5.0; 0.0];   [0.2; 0.2];           1e-4;   1e-6;    'RelCartBias';  'Target';  [ 0.5;  0.1];	 [ 0.015;10.15];    'KF'};
agentParam(3,:) = {'Dubins';    [ 0.2; 0.5;  0.0];        [0.02; 0.02; 0.2];	1e-4;   1e-6;    'RelCartBias';  'Target';  [ 0.0; -0.5];	 [ 0.55;  0.85];    'KF'};


%--- Guessed Target (and bias) Setting ----
%                 Target 1 Dyn.Spec Target 2 Dyn. Spec   Target 1 guess            Target 2 guess              Bias guess       P                        bias Q            target 1 process Q   target 2 process Q       
estiParam(1,:) = {'Linear';         'Linear';           [ 5.0;  0.1; 3.0; 0.5];   [-5.0;  0.0;-0.2; 0.0];     [0.0;0.2];       10*eye(2+SIM.nTarget*4); [0.03;0.03];      [1.5;0.5];            [1.5;0.5]};
estiParam(2,:) = {'Linear';         'Linear';           [ 5.0;  0.1; 3.0; 0.5];   [-5.0;  0.0;-0.2; 0.0];     [0.3;0.1];       10*eye(2+SIM.nTarget*4); [0.05;0.05];      [1.5;1.0];            [1.5;1.0]};
estiParam(3,:) = {'Linear';         'Linear';           [ 5.0;  0.1; 3.0; 0.5];   [-5.0;  0.0;-0.2; 0.0];     [0.2;0.5];       10*eye(2+SIM.nTarget*4); [0.55;0.01];      [1.8;0.3];            [1.8;0.3]};


for iTarget = 1 : SIM.nTarget
    targetSpec = targetParam(iTarget,1);
    TARGET(iTarget) = Target(iTarget, targetSpec{:});
    TARGET(iTarget).DYNAMICS.Initialize(cell2mat(targetParam(iTarget,2)));
    TARGET(iTarget).DYNAMICS.SetParameters(diag(cell2mat(targetParam(iTarget,3))), cell2mat(targetParam(iTarget,4)), cell2mat(targetParam(iTarget,5)));
end


for iAgent = 1 : SIM.nAgent
    agentSpec = agentParam(iAgent,1);
    agentSens = agentParam(iAgent,6);
    agentEsti = agentParam(iAgent,10);
    agentSobj = agentParam(iAgent,7);
    
    AGENT(iAgent) = Agent(iAgent, TARGET, SIM, CLOCK, agentSpec{:}, agentSens{:}, agentEsti{:});
    
    AGENT(iAgent).DYNAMICS.Initialize(cell2mat(agentParam(iAgent,2)));
    AGENT(iAgent).DYNAMICS.SetParameters(diag(cell2mat(agentParam(iAgent,3))), cell2mat(agentParam(iAgent,4)), cell2mat(agentParam(iAgent,5)));
    
    AGENT(iAgent).SENSOR.Initialize(AGENT(1).id);
    AGENT(iAgent).SENSOR.SetParameters(agentSobj{:},cell2mat(agentParam(iAgent,8)),diag(cell2mat(agentParam(iAgent,9)))); % Track Object / bias / R
end


%--- Network class initialization ----
NET = DiskModelNetwork();
NET.InitializeNetwork(SIM.nAgent,20);

%--- Centralized KF subclass initialization ----
% CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
for iAgent = 1 : SIM.nAgent
    AGENT(iAgent).ESTIMATOR.Initialize(...
        [estiParam(iAgent,1);estiParam(iAgent,2)],{cell2mat(estiParam(iAgent,5));cell2mat(estiParam(iAgent,3));cell2mat(estiParam(iAgent,4))},...
        cell2mat(estiParam(iAgent,6)),{diag(cell2mat(estiParam(iAgent,7)));diag(cell2mat(estiParam(iAgent,8)));diag(cell2mat(estiParam(iAgent,9)))});
    AGENT(iAgent).ESTIMATOR.SetParameters('local',AGENT(iAgent).id);
end

%% MAIN PROCEDURE %%%%

% random seed fixing
rng(SIM.sRandom);

% for test.. should be removed.
load('AccelerationInput.mat');
%AccInput(5:6,:) = 0.5*AccInput(1:2,:);
%AccInput(7:8,:) = 0.8*AccInput(3:4,:);

AccInput(5,1:200) = 5; % m/s
AccInput(6,1:200) = 1; % rad/s


for iClock = 1 : CLOCK.nt
    
    %--- clock update ----
    CLOCK.ct = iClock;
    
    %--- Task Allocation (NULL) ----
    for iAgent = 1 : SIM.nAgent
       % AGENT(iAgent).TA.TaskProcedure(); 
    end
    
    %--- Control Decision ----
    for iAgent = 1 : SIM.nAgent
        % for test.. should be removed.
        AGENT(iAgent).CONTROL.u = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.u(iClock,:) = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.stamp(iClock) = CLOCK.ct;
        % AGENT(iAgent).CONTROL.AgentControl(clock,target,environment);
    end
    
    %--- Propagate Target ----
    % target.dynamics :: nonlinear / linear model (for kalman filter)
    % target.update :: update with respect to time
    for iTarget = 1 : SIM.nTarget
        TARGET(iTarget).DYNAMICS.PropagateState(TARGET(iTarget).CONTROL.u, CLOCK);
    end
    
%     % State predction (for testing)
%     for iter = 1 : 10
%         PredictedNoise(iter,:) = AGENT(1).DYNAMICS.MakeNoise('Gaussian');
%     end
%     AGENT(1).DYNAMICS.PredictState(AGENT(1).DYNAMICS.x, AccInput(1:2,iClock:iClock+10), PredictedNoise, CLOCK);
    
    %--- Propagate Agent ----
    for iAgent = 1 : SIM.nAgent
        AGENT(iAgent).DYNAMICS.PropagateState(AGENT(iAgent).CONTROL.u, CLOCK);
    end
    
    %--- Propagate Environment ----
    ENV.UpdateEnvironmentDynamics(CLOCK);
    
    %--- Measurement ----
    for iAgent = 1 : SIM.nAgent
        AGENT(iAgent).SENSOR.Measure(AGENT(iAgent).DYNAMICS.GetPosition(),TARGET,ENV.LANDMARK,CLOCK.ct);
    end
    
    %--- Filter Update :: Centralized KF ----
%    CENTRAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT,CLOCK,'central');
    
    %--- Filter Update :: Local KF ----
    for iAgent = 1 : SIM.nAgent
        
        % Local KF Process
        AGENT(iAgent).ESTIMATOR.TakeProcess(AGENT(iAgent).SENSOR,CLOCK);
        
        % FDDF KF Process :: same procedure as Local KF, but it uses fused
        % estimated data (Xhat, Phat) from the communication.
        % The first iteration is the same as Local KF.
        %        AGENT(iAgent).FDDF_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'fDDF');
    end
  

    %--- Communicate ----
    %--- Make Package and send this to the Network Class (not physical class!) for each agent ----
    for iAgent = 1 : SIM.nAgent
        Z = AGENT(iAgent).COMM.SendPackage(AGENT(iAgent));
        NET.ReceivePackage(Z);
    end
    
    %--- Set Network Graph under Network Class ---
    NET.ComputeProbMatrix();
    NET.ComputeNetworkGraph();
    
    %--- Receive Data from Network Class ----
    for iAgent = 1 : SIM.nAgent
        Z = NET.SendPackage(iAgent);
        AGENT(iAgent).COMM.ReceivePackage(Z);
    end
    
    %--- DDF Information Fusion (managing xhat and Phat) ----
 %    if rem(iClock,CLOCK.delt.FDDF) == 0
 %       for iAgent = 1 : SIMULATION.nAgent
 %           AGENT(iAgent).FDDF.DataFusion(AGENT(iAgent), SIMULATION, NETWORK, CLOCK, 'MMNB');
 %       end
 %    end
    
    fprintf('iteration = %d\n',iClock);
    
end

%% PLOT %%%%
SIM.Plot(AGENT,TARGET,ENV,CLOCK);
