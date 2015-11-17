
close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 3;
nTarget = 2;
SIMULATION = Simulation(nAgent,nTarget);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 200;
CLOCK = Clock(t0,dt,nt);

%--- Environment Classes Setting ----
ENVIRONMENT = Environment(clock);

%--- Target Classes Setting ----
for iTarget = 1 : SIMULATION.nTarget
    TARGET(iTarget) = Target(CLOCK, iTarget);
end

%--- Agent Classes Setting ----
for iAgent = 1 : SIMULATION.nAgent
    AGENT(iAgent) = Agent(TARGET, ENVIRONMENT, SIMULATION, CLOCK , iAgent);
end

%--- Individual TARGET CLASS setting ----
TARGET(1).x = [1.0,-0.1,1.0,0.1]';
TARGET(1).bKFx = [1 1 1 1];
TARGET(1).hist.x = TARGET(1).x;
TARGET(1).hist.stamp = 0;

TARGET(2).x = [-1.0,0.1,-1.0,-0.1]';
TARGET(2).bKFx = [1 1 1 1];
TARGET(2).hist.x = TARGET(2).x;
TARGET(2).hist.stamp = 0;

TARGET(1).Qt = diag([0.2; 0.2]);     
TARGET(2).Qt = diag([0.2; 0.2]);

%--- Individual AGENT CLASS setting ----
AGENT(1).s = [0.3,0.1,-2.5,0,2, 0]';
AGENT(1).bKFs = [1 1 0 0 0 0];
AGENT(1).hist.s = AGENT(1).s;
AGENT(1).hist.stamp = 0;

AGENT(2).s = [0.2,0.5,1.5,0,-5, 0]';
AGENT(2).bKFs = [1 1 0 0 0 0];
AGENT(2).hist.s = AGENT(2).s;
AGENT(2).hist.stamp = 0;

AGENT(3).s = [0.4,0.3,2.5,0,-3, 0]';
AGENT(3).bKFs = [1 1 0 0 0 0];
AGENT(3).hist.s = AGENT(3).s;
AGENT(3).hist.stamp = 0;

AGENT(1).MEASURE.Rp = diag([50.15 1.15]);
AGENT(2).MEASURE.Rp = diag([1.15 50.15]);
AGENT(3).MEASURE.Rp = diag([20.15 1.15]);

AGENT(1).MEASURE.Rt{1} = diag([0.085; 5.85]); % relative target 1 - agent 1
AGENT(2).MEASURE.Rt{1} = diag([5.85; 0.085]); % relative target 1 - agent 2
AGENT(3).MEASURE.Rt{1} = diag([0.085; 5.85]); % relative target 1 - agent 3

AGENT(1).MEASURE.Rt{2} = diag([5.85; 0.085]); % relative target 2 - agent 1
AGENT(2).MEASURE.Rt{2} = diag([0.085; 5.85]); % relative target 2 - agent 2
AGENT(3).MEASURE.Rt{2} = diag([5.85; 0.085]); % relative target 2 - agent 3

%--- Centralized KF subclass initialization ----
SIMULATION.CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
for iAgent = 1 : SIMULATION.nAgent
    SIMULATION.iAgent = iAgent;
    AGENT(iAgent).LOCAL_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'local');
    AGENT(iAgent).FDDF_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'fDDF');
    AGENT(iAgent).FDDF = FactorDDF(AGENT(iAgent),SIMULATION);
end

%% MAIN PROCEDURE %%%%

% for test.. should be removed.
load('AccelerationInput.mat');
AccInput(5:6,:) = 0.5*AccInput(1:2,:);

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
        TARGET(iTarget).UpdateTargetDynamics(CLOCK,SIMULATION.sRandom);
    end
    
    %--- Propagate Agent ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).UpdateAgentDynamics(CLOCK,SIMULATION.sRandom);
    end
    
    %--- Propagate Environment ----
    ENVIRONMENT.UpdateEnvironmentDynamics(CLOCK,SIMULATION.sRandom);
    
    %--- Measurement ----
    for iAgent = 1 : SIMULATION.nAgent
       AGENT(iAgent).MEASURE.TakeMeasurement(AGENT(iAgent),TARGET,ENVIRONMENT,CLOCK,SIMULATION.sRandom); 
    end
    
    %--- Filter Update :: Centralized KF ----
    SIMULATION.CENTRAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT,CLOCK,'central');
    
    %--- Filter Update :: Individual KF :: Local KF and FDDF aided KF ----
    for iAgent = 1 : SIMULATION.nAgent
        % Local KF Process
        AGENT(iAgent).LOCAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'local');
        
        % FDDF KF Process :: same procedure as Local KF, but it uses fused
        % estimated data (Xhat, Phat) from the communication.
        % The first iteration is the same as Local KF.
        AGENT(iAgent).FDDF_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'fDDF');
    end
    
    %--- Communicate ----
    for iAgent = 1 : SIMULATION.nAgent
       AGENT(iAgent).COMM.CommunicationProcedure(AGENT, SIMULATION, AGENT(iAgent).id); 
    end
   
    %--- DDF Information Fusion (managing xhat and Phat) ----
    for iAgent = 1 : SIMULATION.nAgent
       AGENT(iAgent).FDDF.DataFusion(AGENT(iAgent), SIMULATION, CLOCK, 'MMSE');
    end
    
    iClock
    
end

%% PLOT %%%%
SIMULATION.Plot(AGENT,TARGET,CLOCK);
