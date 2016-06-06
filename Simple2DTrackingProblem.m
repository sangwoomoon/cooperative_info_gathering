
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
SIMULATION = Simulation(nAgent,nTarget,nLandMark);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 30;
FDDFt = 1.0;
CLOCK = Clock(t0,dt,nt,FDDFt);

%--- Environment Classes Setting ----
ENVIRONMENT = Environment(CLOCK);

%--- Target Classes Setting ----
TARGET(1) = Target(ENVIRONMENT, 1, 'Linear');
TARGET(2) = Target(ENVIRONMENT, 2, 'Dubins');

%--- Agent Classes Setting ----
AGENT(1) = Agent(TARGET, ENVIRONMENT, SIMULATION, CLOCK, 1, 'Linear'); % linear model
AGENT(2) = Agent(TARGET, ENVIRONMENT, SIMULATION, CLOCK, 2, 'LinearBias'); % linear model w/ sensor bias
AGENT(3) = Agent(TARGET, ENVIRONMENT, SIMULATION, CLOCK, 3, 'Dubins'); % unicycle model

%--- Individual TARGET CLASS setting ----
TARGET(1).DYNAMICS.InitializeState([1.0,-0.1,1.0,0.1]');
TARGET(1).DYNAMICS.SetParameters([1 1 1 1], diag([0.2; 0.2]), 1e-4, 1e-6); % parameter setting order : bKFx, Q, RelTol, AbsTol (last two is for ODE45)

TARGET(2).DYNAMICS.InitializeState([-1.0,0.1,0]');
TARGET(2).DYNAMICS.SetParameters([1 1 1], diag([0.2; 0.2; 0.2]), 1e-4, 1e-6);

%--- Individual AGENT CLASS setting ----
AGENT(1).DYNAMICS.InitializeState([-5.5,0,5, 0]');
AGENT(1).DYNAMICS.SetParameters([0 0 0 0], diag([0.2; 0.2]), 1e-4, 1e-6);

AGENT(2).DYNAMICS.InitializeState([-0.3,0.4,1.5,0,-5, 0]');
AGENT(2).DYNAMICS.SetParameters([1 1 0 0 0 0], diag([0.2; 0.2; 0.2; 0.2]), 1e-4, 1e-6);

AGENT(3).DYNAMICS.InitializeState([0.2,0.5,0]');
AGENT(3).DYNAMICS.SetParameters([0 0 0], diag([0.2; 0.2; 0.2]), 1e-4, 1e-6);
 
for iTarget = 1 : SIMULATION.nTarget
    AGENT(1).MEASURE(iTarget).Rp = diag([1.15 0.15]);
    AGENT(2).MEASURE(iTarget).Rp = diag([0.15 1.15]);
    AGENT(3).MEASURE(iTarget).Rp = diag([1.15 0.15]);
end

AGENT(1).MEASURE(1).Rt = diag([0.085; 2]); % relative target 1 - agent 1
AGENT(2).MEASURE(1).Rt = diag([2; 0.085]); % relative target 1 - agent 2
AGENT(3).MEASURE(1).Rt = diag([0.5; 0.5]); % relative target 1 - agent 3

AGENT(1).MEASURE(2).Rt = diag([2; 0.0085]); % relative target 2 - agent 1
AGENT(2).MEASURE(2).Rt = diag([0.0085; 2]); % relative target 2 - agent 2
AGENT(3).MEASURE(2).Rt = diag([0.5; 0.5]); % relative target 2 - agent 3

%--- Network class initialization ----
NETWORK = Network(inf);

%--- Centralized KF subclass initialization ----
% CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
% for iAgent = 1 : SIMULATION.nAgent
%     SIMULATION.iAgent = iAgent;
%     AGENT(iAgent).LOCAL_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'local');
%     AGENT(iAgent).FDDF_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'fDDF');
%     AGENT(iAgent).FDDF = FactorDDF(AGENT(iAgent),SIMULATION);
% end

%% MAIN PROCEDURE %%%%

% random seed fixing
rng(SIMULATION.sRandom);

% for test.. should be removed.
load('AccelerationInput.mat');
%AccInput(5:6,:) = 0.5*AccInput(1:2,:);
%AccInput(7:8,:) = 0.8*AccInput(3:4,:);

AccInput(5,1:60) = 5; % m/s
AccInput(6,1:60) = 1; % rad/s

for iClock = 1 : CLOCK.nt
    
    %--- clock update ----
    CLOCK.ct = iClock;
    
    %--- Task Allocation (NULL) ----
    for iAgent = 1 : SIMULATION.nAgent
       % AGENT(iAgent).TA.TaskProcedure(); 
    end
    
    %--- Control Decision ----
    for iAgent = 1 : SIMULATION.nAgent
        % for test.. should be removed.
        AGENT(iAgent).CONTROL.u = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.u(iClock,:) = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.stamp(iClock) = CLOCK.ct;
        % AGENT(iAgent).CONTROL.AgentControl(clock,target,environment);
    end
    
    %--- Propagate Target ----
    % target.dynamics :: nonlinear / linear model (for kalman filter)
    % target.update :: update with respect to time
    for iTarget = 1 : SIMULATION.nTarget
        TARGET(iTarget).DYNAMICS.TimeUpdate(TARGET(iTarget).CONTROL.u,CLOCK);
    end
    
    %--- Propagate Agent ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).DYNAMICS.TimeUpdate(AGENT(iAgent).CONTROL.u,CLOCK); % UpdateAgentDynamics -> TimeUpdate
    end
    
    %--- Propagate Environment ----
    ENVIRONMENT.UpdateEnvironmentDynamics(CLOCK,SIMULATION.sRandom);
    
    %--- Measurement ----
%     for iAgent = 1 : SIMULATION.nAgent
%         for iTarget = 1 : SIMULATION.nTarget
%             AGENT(iAgent).MEASURE(iTarget).TakeMeasurement(AGENT(iAgent),TARGET(iTarget),ENVIRONMENT,CLOCK,SIMULATION.sRandom);
%         end
%     end
    
    %--- Filter Update :: Centralized KF ----
%    CENTRAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT,CLOCK,'central');
    
    %--- Filter Update :: Individual KF :: Local KF and FDDF aided KF ----
%    for iAgent = 1 : SIMULATION.nAgent
        % Local KF Process
%        AGENT(iAgent).LOCAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'local');
        
        % FDDF KF Process :: same procedure as Local KF, but it uses fused
        % estimated data (Xhat, Phat) from the communication.
        % The first iteration is the same as Local KF.
%        AGENT(iAgent).FDDF_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'fDDF');
%    end
  
    %--- Innovation ---
    
    
    %--- Communicate ----
    %--- Send Data to Network Class ----
%    for iAgent = 1 : SIMULATION.nAgent
%       AGENT(iAgent).COMM.SendData(NETWORK,AGENT(iAgent),SIMULATION); 
%    end
    
    %--- Set Network Graph under Network Class ---
%    NETWORK.DetermineCommStatus(SIMULATION,AGENT);
    
    %--- Receive Data from Network Class ----
%    for iAgent = 1 : SIMULATION.nAgent
%       AGENT(iAgent).COMM.ReceiveData(NETWORK,AGENT(iAgent),SIMULATION); 
%    end
    
    %--- DDF Information Fusion (managing xhat and Phat) ----
 %    if rem(iClock,CLOCK.delt.FDDF) == 0
 %       for iAgent = 1 : SIMULATION.nAgent
 %           AGENT(iAgent).FDDF.DataFusion(AGENT(iAgent), SIMULATION, NETWORK, CLOCK, 'MMNB');
 %       end
 %    end
    
    fprintf('iteration = %d\n',iClock);
    
end

%% PLOT %%%%
SIMULATION.Plot(AGENT,TARGET,CLOCK);
% SIMULATION.Plot(AGENT,TARGET,CENTRAL_KF,CLOCK);
