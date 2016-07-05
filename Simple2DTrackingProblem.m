
% range - bearing error / bias error is not added : EKF

close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 5;
nTarget = 10;
% nLandMark = 0;

SIMULATION = Simulation(nAgent,nTarget);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 200;
FDDFt = 0.1;
CLOCK = Clock(t0,dt,nt,FDDFt);

%--- Environment Classes Setting ----
ENVIRONMENT = Environment(clock,[-1500,1500,-1500,1500]);

%--- Target Classes Setting ----
for iTarget = 1 : SIMULATION.nTarget
    TARGET(iTarget) = Target(CLOCK, ENVIRONMENT, iTarget, SIMULATION.sRandom);
end

%--- Agent Classes Setting ----
for iAgent = 1 : SIMULATION.nAgent
    AGENT(iAgent) = Agent(TARGET, ENVIRONMENT, SIMULATION, CLOCK, iAgent);
end

%--- Individual Agent Measurement Setting (identical setting) ----
for iAgent = 1 : SIMULATION.nAgent
    AGENT(iAgent).MEASURE.Rt = diag([50^2 (10*pi/180)^2]); % relative target 1 - agent 1
end

%--- Centralized KF subclass initialization ----
% SIMULATION.CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
for iAgent = 1 : SIMULATION.nAgent
    SIMULATION.iAgent = iAgent;
    for iTarget = 1 : SIMULATION.nTarget
        LOCAL_KF(iTarget) = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET(iTarget),CLOCK,'local');
    end
    AGENT(iAgent).LOCAL_KF = LOCAL_KF;
end

%% MAIN PROCEDURE %%%%

for iClock = 1 : CLOCK.nt
    
    %--- Voronoi Partition ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).TA.TakeVoronoi(AGENT(iAgent), ENVIRONMENT, SIMULATION );
        AGENT(iAgent).TA.TakeTargetID(AGENT(iAgent), SIMULATION);
    end
    
    %--- to do :: control input coding (for motion input)
    
    
    %--- clock update ----
    CLOCK.ct = iClock; % started from zero (directly uses the initial conditioned data)
    
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
        
    %--- Task Allocation from Voronoi cell ----
    for iAgent = 1 : SIMULATION.nAgent
       % AGENT(iAgent).TA.TaskProcedure(); 
    end
    
    %--- Measurement (single target measuring) ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).MEASURE.TakeMeasurement(AGENT(iAgent),TARGET(AGENT(iAgent).TA.TrackID),ENVIRONMENT,CLOCK,SIMULATION.sRandom);
    end
    
    %--- Communicate ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).COMM.ComputeBeta(AGENT, SIMULATION, AGENT(iAgent).id);
        AGENT(iAgent).COMM.ComputeCommArray();
        AGENT(iAgent).COMM.CommunicationProcedure(AGENT, SIMULATION);
    end
    
    %--- Filter Update :: Centralized KF ----
%     SIMULATION.CENTRAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT,CLOCK,'central');
    
    %--- Filter Update :: Individual KF :: Local KF and FDDF aided KF ----
    for iAgent = 1 : SIMULATION.nAgent
        % Local KF Process
        for iTarget = 1 : SIMULATION.nTarget
            AGENT(iAgent).LOCAL_KF(iTarget).ExtendedKalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),TARGET(iTarget), CLOCK,'local');
        end
    end

    clf;
    AGENT(1).TA.Plot();
    for iter = 1 : 10
        TARGET(iter).Plot();
    end
    
    fprintf('iteration = %d\n',CLOCK.ct);
    
end

%% PLOT %%%%
SIMULATION.Plot(AGENT,TARGET,CLOCK);
