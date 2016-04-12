
% range - bearing error / bias error is not added : EKF

close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 2;
nTarget = 3;
% nLandMark = 0;

SIMULATION = Simulation(nAgent,nTarget);

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 100;
FDDFt = 0.1;
CLOCK = Clock(t0,dt,nt,FDDFt);

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
TARGET(1).x = [10.0,0,10.0,0]'; % stationary
TARGET(1).bKFx = [1 1 1 1];
TARGET(1).bLandMark = 0;
TARGET(1).hist.x = TARGET(1).x;
TARGET(1).hist.stamp = 0;

TARGET(2).x = [-10.0,0,-10.0,0]'; % stationary
TARGET(2).bKFx = [1 1 1 1];
TARGET(2).bLandMark = 0;
TARGET(2).hist.x = TARGET(2).x;
TARGET(2).hist.stamp = 0;

TARGET(3).x = [5.0,0,5.0,0]'; % stationary
TARGET(3).bKFx = [1 1 1 1];
TARGET(3).bLandMark = 0;
TARGET(3).hist.x = TARGET(3).x;
TARGET(3).hist.stamp = 0;

% Q is not usable if it is for landmark.
TARGET(1).Qt = diag([0.05; 0.05]);     
TARGET(2).Qt = diag([0.05; 0.05]);
% TARGET(3).Qt = diag([0.2; 0.2]); % this value is not usable 

%--- Individual AGENT CLASS setting ----
AGENT(1).s = [-2.5,0,2, 0]';
% AGENT(1).bKFs = [1 1 0 0 0 0];
AGENT(1).hist.s = AGENT(1).s;
AGENT(1).hist.stamp = 0;

AGENT(2).s = [1.5,0,-5, 0]';
% AGENT(2).bKFs = [1 1 0 0 0 0];
AGENT(2).hist.s = AGENT(2).s;
AGENT(2).hist.stamp = 0;

% AGENT(3).s = [0.3,0.2,2.5,0,-3, 0]';
% AGENT(3).bKFs = [1 1 0 0 0 0];
% AGENT(3).hist.s = AGENT(3).s;
% AGENT(3).hist.stamp = 0;
 
% AGENT(4).s = [0.3,0.2,3.5,0,-2, 0]';
% AGENT(4).bKFs = [1 1 0 0 0 0];
% AGENT(4).hist.s = AGENT(3).s;
% AGENT(4).hist.stamp = 0;

% for iTarget = 1 : SIMULATION.nTarget
%     AGENT(1).MEASURE(iTarget).Rp = diag([5^2 (10*pi/180)^2]);
%     AGENT(2).MEASURE(iTarget).Rp = diag([5^2 (10*pi/180)^2]);
% %     AGENT(3).MEASURE(iTarget).Rp = diag([1.15 0.15]);
% %     AGENT(4).MEASURE(iTarget).Rp = diag([0.15 2.15]);
% end

AGENT(1).MEASURE(1).Rt = diag([5^2 (10*pi/180)^2]); % relative target 1 - agent 1
AGENT(2).MEASURE(1).Rt = diag([5^2 (10*pi/180)^2]); % relative target 1 - agent 2
% AGENT(3).MEASURE(1).Rt = diag([0.5; 0.5]); % relative target 1 - agent 3
% AGENT(4).MEASURE(1).Rt = diag([2; 2]); % relative target 1 - agent 4

AGENT(1).MEASURE(2).Rt = diag([5^2 (10*pi/180)^2]); % relative target 2 - agent 1
AGENT(2).MEASURE(2).Rt = diag([5^2 (10*pi/180)^2]); % relative target 2 - agent 2
% AGENT(3).MEASURE(2).Rt = diag([0.5; 0.5]); % relative target 2 - agent 3
% AGENT(4).MEASURE(2).Rt = diag([2; 2]); % relative target 2 - agent 4

AGENT(1).MEASURE(3).Rt = diag([5^2; (10*pi/180)^2]); % relative target 3 - agent 1 (almost exactly knows)
AGENT(2).MEASURE(3).Rt = diag([5^2; (10*pi/180)^2]); % relative target 3 - agent 2 (almost exactly knows)
% AGENT(3).MEASURE(3).Rt = diag([2; 2]); % relative target 3 - agent 3 (bad measurement)
% AGENT(4).MEASURE(3).Rt = diag([2; 2]); % relative target 3 - agent 4 (bad measurement)


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

% for test.. should be removed.
load('AccelerationInput.mat');
AccInput(5:6,:) = 0.5*AccInput(1:2,:);
AccInput(7:8,:) = 0.8*AccInput(3:4,:);

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
