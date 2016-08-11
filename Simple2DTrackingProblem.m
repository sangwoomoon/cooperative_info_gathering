
close all;
clear all;
clc;
format compact;
hold on;

%% INITIAL SETTING %%%%

%--- Simulation Class Setting ----
nAgent = 3;
nTarget = 3;
nLandMark = 1;
SIM = Simulation(nAgent,nTarget,nLandMark,'KF','Disk');

%--- Clock Class Setting ----
t0 = 0.1;
dt = 0.1;
nt = 200;
FDDFt = 1.0;
CLOCK = Clock();
CLOCK.Initialize(t0,dt,nt,FDDFt);

% Import input files
[ agentParam, targetParam, landmarkParam, centEstiParam, locEstiParam, networkParam ] = SIM.ImportInput();

% Initialize simulation
[ AGENT, TARGET, ENV] = SIM.Initialize(agentParam, targetParam, landmarkParam, centEstiParam, locEstiParam, networkParam );

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
    
    %--- Propagate Target ----
    % target.dynamics :: nonlinear / linear model (for kalman filter)
    % target.update :: update with respect to time
    for iTarget = 1 : SIM.nTarget
        TARGET(iTarget).DYNAMICS.PropagateState(TARGET(iTarget).CONTROL.u, CLOCK);
    end
    
    %--- Propagate Environment ----
    ENV.UpdateEnvironmentDynamics(CLOCK);
    
    %--- AGENT PART -----
    for iAgent = 1 : SIM.nAgent
        
        %--- Control Profile Input ----
        % for test.. should be removed.
        AGENT(iAgent).CONTROL.u = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.u(iClock,:) = AccInput(2*(iAgent-1)+1:2*iAgent,iClock);
        AGENT(iAgent).CONTROL.hist.stamp(iClock) = CLOCK.ct;
        % AGENT(iAgent).CONTROL.AgentControl(clock,target,environment);
    
        %--- Propagate Agent ----
        AGENT(iAgent).DYNAMICS.PropagateState(AGENT(iAgent).CONTROL.u, CLOCK);
   
        %--- Sensor Measure ----
        AGENT(iAgent).SENSOR.Measure(AGENT(iAgent).DYNAMICS.GetPosition(),TARGET,ENV.LANDMARK,CLOCK.ct);
   
        %--- Filter Update :: Local KF ----
        % Local KF Process
        % measurement is treated as cell because of multiple heterogeneous
        % sensor classes (especially compatible with centralized scheme)
        AGENT(iAgent).ESTIMATOR.TakeProcess({AGENT(iAgent).SENSOR.meas},CLOCK);
    end

    %--- Communicate ----
    SIM.NETWORK.DeliverPackage(AGENT);
    
    %--- DDF Information Fusion (managing xhat and Phat) ----
    if rem(iClock,CLOCK.delt.FDDF) == 0
       for iAgent = 1 : SIM.nAgent
           AGENT(iAgent).FUSION.TakeProcess(AGENT(iAgent).ESTIMATOR.xhat, AGENT(iAgent).ESTIMATOR.Phat,...
               AGENT(iAgent).COMM.Z, AGENT(iAgent).SENSOR.bTrack, CLOCK.ct, 'diag');
           AGENT(iAgent).FUSION.AllocateFusionData(AGENT(iAgent).ESTIMATOR);
       end
       
       fprintf('fusion process step = %d\n',iClock);
    end
    
    %--- Filter Update :: Centralized KF ----
    for iAgent = 1 : SIM.nAgent
        % in order to implement for heterogeneous type of sensors, "cell"
        % array is used
        meas{iAgent} = AGENT(iAgent).SENSOR.meas; % gather sensor class only (not affected by inserting "AGENT.SENSOR")
    end
    SIM.ESTIMATOR.TakeProcess(meas,CLOCK); % whole sensor classes 
    
    
    %--- clock update ----
    CLOCK.ct = iClock;
    
end

%% PLOT %%%%
SIM.Plot(AGENT,TARGET,ENV,CLOCK);







%     % State predction (for testing)
%     for iter = 1 : 10
%         PredictedNoise(iter,:) = AGENT(1).DYNAMICS.MakeNoise('Gaussian');
%     end
%     AGENT(1).DYNAMICS.PredictState(AGENT(1).DYNAMICS.x, AccInput(1:2,iClock:iClock+10), PredictedNoise, CLOCK);
    
