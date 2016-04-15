
% range - bearing error / bias error is not added : EKF

close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 5;
nTarget = 5;
% nLandMark = 0;

SIMULATION = Simulation(nAgent,nTarget);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 100;
FDDFt = 0.1;
CLOCK = Clock(t0,dt,nt,FDDFt);

%--- Voronoi Class (under SIM) Setting ----
SIMULATION.VORONOI = Voronoi(CLOCK);

%--- Lloyd Class (under SIM) setting ----
SIMULATION.LLOYD = Lloyd(CLOCK);

%--- Environment Classes Setting ----
ENVIRONMENT = Environment(clock);

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
    for iTarget = 1 : SIMULATION.nTarget
        AGENT(iAgent).MEASURE(iTarget).Rt = diag([5^2 (10*pi/180)^2]); % relative target 1 - agent 1
    end
end

%--- Centralized KF subclass initialization ----
% SIMULATION.CENTRAL_KF = KalmanFilter(SIMULATION,AGENT,TARGET,CLOCK,'central'); 

%--- Individaulized KF subclass initialization ----
for iAgent = 1 : SIMULATION.nAgent
    SIMULATION.iAgent = iAgent;
    AGENT(iAgent).LOCAL_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'local');
%     AGENT(iAgent).FDDF_KF = KalmanFilter(SIMULATION,AGENT(iAgent),TARGET,CLOCK,'fDDF');
%     AGENT(iAgent).FDDF = FactorDDF(AGENT(iAgent),SIMULATION);
end

%% MAIN PROCEDURE %%%%

for iClock = 1 : CLOCK.nt
    
    %--- clock update ----
    CLOCK.ct = iClock;
    
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
    
    %--- Voronoi Partition ----
    SIMULATION.VORONOI.TakeVoronoi(AGENT, ENVIRONMENT, SIMULATION );
    
    %--- Task Allocation from Voronoi cell ----
    for iAgent = 1 : SIMULATION.nAgent
       % AGENT(iAgent).TA.TaskProcedure(); 
    end
    
    %--- Measurement ----
    for iAgent = 1 : SIMULATION.nAgent
        for iTarget = 1 : SIMULATION.nTarget
            AGENT(iAgent).MEASURE(iTarget).TakeMeasurement(AGENT(iAgent),TARGET(iTarget),ENVIRONMENT,CLOCK,SIMULATION.sRandom);
        end
    end
    
    %--- Filter Update :: Centralized KF ----
%     SIMULATION.CENTRAL_KF.KalmanFilterAlgorithm(SIMULATION,AGENT,CLOCK,'central');
    
    %--- Filter Update :: Individual KF :: Local KF and FDDF aided KF ----
    for iAgent = 1 : SIMULATION.nAgent
        % Local KF Process
        AGENT(iAgent).LOCAL_KF.ExtendedKalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),TARGET, CLOCK,'local');
        
        % FDDF KF Process :: same procedure as Local KF, but it uses fused
        % estimated data (Xhat, Phat) from the communication.
        % The first iteration is the same as Local KF.
%         AGENT(iAgent).FDDF_KF.KalmanFilterAlgorithm(SIMULATION,AGENT(iAgent),CLOCK,'fDDF');
    end
    
    %--- Lloyd's Algorithm :: Compute Centroid
    SIMULATION.LLOYD.ComputeCentroid(SIMULATION);
    SIMULATION.LLOYD.AllocateCentroid(AGENT);
    
    %--- One-step Ahead Control Decision ----
    for iAgent = 1 : SIMULATION.nAgent
        AGENT(iAgent).CONTROL.ComputeInput(AGENT(iAgent),'Lloyd');
    end
    
    %--- Communicate ----
%     for iAgent = 1 : SIMULATION.nAgent
%        AGENT(iAgent).COMM.CommunicationProcedure(AGENT, SIMULATION, AGENT(iAgent).id); 
%     end
   
    %--- DDF Information Fusion (managing xhat and Phat) ----
%      if rem(iClock,CLOCK.delt.FDDF) == 0
%         for iAgent = 1 : SIMULATION.nAgent
%             AGENT(iAgent).FDDF.DataFusion(AGENT(iAgent), SIMULATION, CLOCK, 'MMNB');
%         end
%      end
    
    fprintf('iteration = %d\n',iClock);
    
end

%% PLOT %%%%
SIMULATION.Plot(AGENT,TARGET,CLOCK);
