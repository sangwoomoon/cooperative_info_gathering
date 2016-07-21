
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
ENV = Environment(CLOCK);

%--- Target Classes Setting ----
TARGET(1) = Target(1, 'Linear');
TARGET(2) = Target(2, 'Linear');

%--- Agent Classes Setting ----
AGENT(1) = Agent(1, TARGET, SIM, CLOCK, 'Linear', 'RelCartBias','KF'); % linear dynamics, linear measurement, Kalman filter estimation
AGENT(2) = Agent(2, TARGET, SIM, CLOCK, 'Linear', 'RelCartBias','KF'); 
AGENT(3) = Agent(3, TARGET, SIM, CLOCK, 'Dubins', 'RelCartBias','KF'); % unicycle model, linear measurement, Kalman filter estimation

%--- Individual TARGET CLASS setting ----
TARGET(1).DYNAMICS.InitializeState([1.0,-0.5,1.0,0.5]');
TARGET(1).DYNAMICS.SetParameters(diag([1.2; 1.2]), 1e-4, 1e-6); % parameter setting order : bKFx, Q, RelTol, AbsTol (last two is for ODE45)

% TARGET(2).DYNAMICS.InitializeState([-10.0,-5.0,-1.0]'); % Dubins
% TARGET(2).DYNAMICS.SetParameters(diag([1.2; 1.2; 1.2]), 1e-4, 1e-6); % Dubins
TARGET(2).DYNAMICS.InitializeState([-10.0,1.5,-5.0,-1.0]');
TARGET(2).DYNAMICS.SetParameters(diag([1.2; 1.2]), 1e-4, 1e-6);

%--- Individual AGENT CLASS setting :: Dynamics ----
AGENT(1).DYNAMICS.InitializeState([-5.5,0,5, 0]');
AGENT(1).DYNAMICS.SetParameters(diag([0.2; 0.2]), 1e-4, 1e-6);

AGENT(2).DYNAMICS.InitializeState([1.5,0,-5, 0]');
AGENT(2).DYNAMICS.SetParameters(diag([0.2; 0.2]), 1e-4, 1e-6);

AGENT(3).DYNAMICS.InitializeState([0.2,0.5,0]');
AGENT(3).DYNAMICS.SetParameters(diag([0.2; 0.2; 0.2]), 1e-4, 1e-6);
 
%--- Individual AGENT CLASS setting :: Sensor ----
AGENT(1).SENSOR.Initialize(AGENT(1).id);
AGENT(1).SENSOR.SetParameters('Target',[0.2, -0.3]',diag([0.03 0.05]),diag([0.05 0.05]),diag([10.15 0.015]));
AGENT(2).SENSOR.Initialize(AGENT(2).id);
AGENT(2).SENSOR.SetParameters('Target',[0.5; 0.1]',diag([0.05 0.03]),diag([0.05 0.05]),diag([0.015 10.15]));
AGENT(3).SENSOR.Initialize(AGENT(3).id);
AGENT(3).SENSOR.SetParameters('Target',[0; -0.5]',diag([0.10 0.10]),diag([0.05 0.05]),diag([0.55 0.85]));

%--- Network class initialization ----
NET = DiskModelNetwork();
NET.InitializeNetwork(SIM.nAgent,20);

%--- Centralized KF subclass initialization ----
% CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
for iAgent = 1 : SIM.nAgent
    AGENT(iAgent).ESTIMATOR.Initialize([0 0.2, 5.0 0.1 3.0 0.5, -5.0,0,2.0,0]',10*eye(10));
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
        
        % Take matrix for estimation : should be belonged and excuted
        % in the Kalman Filter sub-class (not generalized one!)
        F = AGENT(iAgent).GatherJacobian(TARGET,CLOCK,'state');
        Gamma = AGENT(iAgent).GatherJacobian(TARGET,CLOCK,'noise');
        
        Y = AGENT(iAgent).SENSOR.GatherMeasurements();
        H = AGENT(iAgent).SENSOR.TakeJacobian();
        
        Q = AGENT(iAgent).GatherProcNoiseCovMatrix(TARGET);
        R = AGENT(iAgent).SENSOR.GatherMeasNoiseCovMatrix();
        
        % Set matrix for estimation
        AGENT(iAgent).ESTIMATOR.SetMatrices(Y,F,Gamma,H,Q,R);
        
        % Local KF Process
        AGENT(iAgent).ESTIMATOR.TakeProcess();
        
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
