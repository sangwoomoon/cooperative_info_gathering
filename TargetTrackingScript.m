
% Target Localization Problem Script
% Particle Method based Mutual Information
%
%
%
% X(t+1) ~ P_ta(X(t+1)|X(t))
% Y(t)   ~ P_se(Y(t)|X(t);s(t))
% Z(t)   ~ P_co(Z(t)|Y(t);s^i(t),s^l(t))
%
% - coded by Sangwoo Moon
% - Created:      4/ 3/2018
% - 1st revision: 4/18/2018
% - 2nd revision: 6/25/2018
% - 3rd revision: 9/ 5/2018
% - 4th revision:12/ 8/2018
% - 5th revision: 1/17/2019
% - 6th revision: 8/ 9/2019
%
%   X(t+1) = X(t) + W                               : 2D-static Linear Gaussian
%   Y(t) = {0,1} with respect to Sensing Region     : 2D on the ground, circle region for target detection
%   s(t+1) = f(s(t),u(t))                           : u(t) = [0 -omega +omega]


close all;
clear;
% clc;
format compact;

% --------------------------------------------------------------------
% control parameters for comparison
nSim = 1; % for Monte-Carlo approach with fixed independent condition
nPt = [100 500 1000 2000];
dist = [200 400 600];
nT = [1];
nA = [2 3 5 10];
dRefPt = [1 5 10 25 50];
nSample = [1 100 500 1000];
commAware = [0 1];
planner = {'random','mean','MI','MI_comm'};

% comparison setting
flagCondition  = 'planner';

% simulation by changing independent condition
switch flagCondition
    case 'nPt'
        mSim = length(nPt);
    case 'nT'
        mSim = length(nT);
    case 'dist'
        mSim = length(dist);
    case 'nA'
        mSim = length(nA);
    case 'dRefPt'
        mSim = length(dRefPt);
    case 'nSample'
        mSim = length(nSample);
    case 'commAware'
        mSim = length(commAware);
    case 'planner'
        mSim = length(planner);
    otherwise
        mSim = 1;
end
% --------------------------------------------------------------------


% --------------------------------------------------------------------
% time parameters
nt = 50;
dt = 1;
% --------------------------------------------------------------------


% RandSeed = rng;

% make array of simulation structure
for jSim = 1:mSim
    for iSim = 1:nSim
        
        %-------------------------------
        %   sim setting
        %-------------------------------
        
        
        %----------------------
        % simulation structure
        % in order to allocate as the array of simulation
        sim(jSim,iSim) = InitializeSim(   4,       1,     'MI',       1,           1,       'uniform',        0,         1,     'Pos',  'unicycle',    'bear',   'PF'    );
                                    % nAgent | nTarget | flagDM | flagComm | flagActComm | flagPdfCompute | flagLog | flagPlot | target |  agent     | sensor   | filter
        
        % flagDM         ||   'random': random decision | 'MI': mutual information-based decision | 'mean': particle mean following
        % flagComm       ||   0: perfect communication | 1: imperfect communication and communication awareness
        % flagPdfCompute ||   'uniform': uniformly discretized domain | 'cylinder': cylinder based computation w.r.t particle set
        % flagLog        ||   0: skip logging | 1: log data
        % flagPlot       ||   flag for the display of trajectories and particles evolution
        % target         ||   'Pos': position only | 'PosVel': position and velocity | 'PosRF': position with RF properties (referred by Maciej's JCSD paper)
        % sensor         ||   'linear', 'range_bear', 'detection', 'bear', 'RF'
        %----------------------
        
    end
end


for jSim = 1:mSim
    
    
    switch flagCondition
        case 'dist'
            % with respect to distance between agents
            fprintf('\njSim = %d, distance = %d\n',jSim,dist(jSim));
        case 'nT'
            % with respect to nT
            fprintf('\njSim = %d, nT = %d\n',jSim,nT(jSim));
        case 'nPt'
            % with respect to nPt
            fprintf('\njSim = %d, nPt = %d\n',jSim,nPt(jSim));
        case 'dRefPt'
            % with respect to nDpdf
            fprintf('\njSim = %d, dRefPt = %d\n',jSim,dRefPt(jSim));
        case 'nSample'
            % with respect to nSample
            fprintf('\njSim = %d, nSample = %d\n',jSim,nSample(jSim));
        case 'commAware'
            % with respect to planning w/ comm vs. w/o comm
            fprintf('\njSim = %d, commAware = %d\n',jSim,commAware(jSim));
        case 'planner'
            % with respect to planner scheme: random, mean-following, MI w/o communcation-aware, MI w/ comm-aware
            fprintf('\njSim = %d, planner = %s\n',jSim,planner{jSim});
        case 'nA'
            % with respect to number of agents
            fprintf('\njSim = %d, nA = %d\n',jSim,nA(jSim));
    end
    
    for iSim = 1:nSim
        
        % hard-coded because it should be inside of sim initialization
        switch flagCondition
            case 'commAware'
                % with respect to planning w/ comm vs. w/o comm
                sim(jSim,iSim).flagComm = commAware(jSim);
            case 'planner'
                sim(jSim,iSim).flagDM = planner{jSim};
                if jSim == 3
                    sim(jSim,iSim).flagComm = 0;
                elseif jSim == 4
                    sim(jSim,iSim).flagDM = planner{jSim-1};
                    sim(jSim,iSim).flagComm = 1;
                end
            case 'nA'
                sim(jSim,iSim).nAgent = nA(jSim);
        end
        
        %----------------------
        % clock structure
        sim(jSim,iSim).clock = InitializeClock(  nt  ,  dt  );
        % nt  |  dt
        %----------------------
        
        %----------------------
        % field structure
        sim(jSim,iSim).field = InitializeField(sim(jSim,iSim), [-1000 1000],[-1000 1000]); % overloaded by the number of boundaries (2D)
        % sim(jSim,iSim).field = InitializeField(sim(jSim,iSim), [-300 300],[-300 300],[-300,300]); % overloaded by the number of boundaries (3D)
        %----------------------
        
        %----------------------
        % target structure
        for iTarget = 1:sim(jSim,iSim).nTarget
            sim(jSim,iSim).target(iTarget) = InitializeTarget(iTarget, sim(jSim,iSim));
        end
        %----------------------
        
        %----------------------
        % agent structure
        
        % parameters fleet of agent for initial positioning
        for iAgent = 1:sim(jSim,iSim).nAgent
            switch flagCondition
                case 'dist'
                    sim(jSim,iSim).agent(iAgent) = InitializeAgent(iAgent, sim(jSim,iSim), 10, dist(jSim));
                otherwise
                    sim(jSim,iSim).agent(iAgent) = InitializeAgent(iAgent, sim(jSim,iSim), 10, 0);
            end
        end
        %----------------------
        
        %----------------------
        % sensor structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            % make heterogeneous sensor
            for iTarget = 1:sim(jSim,iSim).nTarget
                sim(jSim,iSim).sensor(iAgent,iTarget) = ...
                    InitializeSensor(sim(jSim,iSim),iAgent,iTarget,   40,    0.9,  sim(jSim,iSim).agent(iAgent), sim(jSim,iSim).target(iTarget), diag([20^2,20^2,20^2]'), diag([5^2,(pi/18)^2]'),  9 );
                                                                    % range | beta |                                                                      R              |       R_rangebear    |  R_rf
            end
        end
        %----------------------
        
        %----------------------
        % communication structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            sim(jSim,iSim).comm(iAgent) = InitializeCommunication(iAgent,sim(jSim,iSim));
        end
        %----------------------
        
        %----------------------
        % filter structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            for iTarget = 1:sim(jSim,iSim).nTarget
                xhat = zeros(length(sim(jSim,iSim).target(iTarget).x),1);
                Phat = diag([50^2,50^2]);
                
                switch flagCondition
                    case 'nPt'
                        
                        switch sim(jSim,iSim).flagTarget
                            case 'PosRF'
                                % based on reference, "Cooperative
                                % Target Localization with a Communication-Aware Unmanned
                                % Aircraft System, Stachura, Maciej and Frew, Eric W.
                                Phat = diag([2e4,2e4,1e-7,2e-4]);
                                Q = diag([10e-2 10e-2 5e-20 5e-9]);
                                sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,  Q, nPt(jSim));
                                                                                                                      %  xhat | Phat|  Q | nPt
                            otherwise
                                sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,   diag([18^2,18^2,18^2]), nPt(jSim));
                                                                                                                        %  xhat | Phat   |            Q         | nPt
                        end
                        
                    otherwise
                        
                        switch sim(jSim,iSim).flagTarget
                            case 'PosRF'
                                % based on reference, "Cooperative
                                % Target Localization with a Communication-Aware Unmanned
                                % Aircraft System, Stachura, Maciej and Frew, Eric W.
                                Phat = diag([2e4,2e4,1e-7,2e-4]);
                                Q = diag([10e-2 10e-2 5e-20 5e-9]);
                                sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,   Q,  nPt(1));
                                                                                                                      %  xhat | Phat  | Q | nPt
                            otherwise
                                sim(jSim,iSim).filter(iAgent,iTarget) = InitializeFilter(sim(jSim,iSim),iAgent,iTarget,  xhat,  Phat,   diag([18^2,18^2,18^2]), nPt(1));
                                                                                                                        %  xhat | Phat   |            Q         | nPt
                        end
                        
                end
            end
        end
        %----------------------
        
        %----------------------
        % planner structure
        for iAgent = 1:sim(jSim,iSim).nAgent
            
            switch flagCondition
                case 'nPt'
                    sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(2),  nPt(jSim), dRefPt(4), 100 );
                                                                                             % dt | nT |     nPt    | dRefPt   | nSample
                case 'dRefPt'
                    sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(2),  nPt(1),    dRefPt(jSim), 100 );
                                                                                            % dt | nT |     nPt    | dRefPt      | nSample
                case 'nSample'
                    sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(2),  nPt(1),    dRefPt(4), nSample(jSim) );
                                                                                            % dt | nT |     nPt    | dRefPt      | nSample
                otherwise
                    switch flagCondition
                        case 'nT'
                            sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 10,  nT(jSim),  nPt(1), dRefPt(4), 5 );
                                                                                                    % dt |     nT   | nPt   | dRefPt   | nSample
                        otherwise
                            sim(jSim,iSim).planner(iAgent) = InitializePlanner(iAgent,sim(jSim,iSim), 3,  nT(1),  nPt(1), dRefPt(4), 5 );
                                                                                                    % dt |   nT |    nPt | dRefPt   | nSample
                    end
            end
        end
        %----------------------
        
        
        %% ---------------------------------
        % Sim Operation
        %-----------------------------------
        
        for iClock = 1:sim(jSim,iSim).clock.nt
            
            %-----------------------------------
            % PF-based Mutual information Computation and decision-making
            %-----------------------------------
            
            % compute future information with respect to action profiles
            % distributed scheme to each agent:
            % COMPUTED INFORMATION IS DIFFERENT WITH RESPECT TO AGENT
            for iAgent = 1:sim(jSim,iSim).nAgent
                
                % decision making procedure
                switch sim(jSim,iSim).flagDM
                    
                    % random decision
                    case 'random'
                        
                        % check the possible action command using geofence
                        bAction = nan(1,sim(jSim,iSim).planner(iAgent).actionNum);
                        for iAction = 1:sim(jSim,iSim).planner(iAgent).actionNum
                            
                            state = UpdateAgentState(sim(jSim,iSim).agent(iAgent).s,sim(jSim,iSim).planner(iAgent).actionSet(iAction),sim(jSim,iSim).clock.dt);
                            if (state(1) > sim(jSim,iSim).field.bufferZone(1) && state(1) < sim(jSim,iSim).field.bufferZone(2)) ...
                                    && (state(2) > sim(jSim,iSim).field.bufferZone(3) && state(2) < sim(jSim,iSim).field.bufferZone(4)) % inside geofence
                                bAction(iAction) = 1;
                            else
                                bAction(iAction) = 0;
                            end
                            
                        end
                        
                        % AD-HOC: when the agent is close to the geofence so that all
                        % cost candidates are infinity, then go to the origin.
                        if sum(bAction) == 0
                            sim(jSim,iSim).planner(iAgent).actIdx = MoveToPoint([0 0], sim(jSim,iSim).agent(iAgent).s);
                        else
                            sim(jSim,iSim).planner(iAgent).actIdx = ceil(rand()*sim(jSim,iSim).planner(iAgent).actionNum);
                        end
                        sim(jSim,iSim).planner(iAgent).input = sim(jSim,iSim).planner(iAgent).actionSet(:,sim(jSim,iSim).planner(iAgent).actIdx);
                        
                        % follow mean of a single target
                    case 'mean'
                        
                        sim(jSim,iSim).planner(iAgent).actIdx = MoveToPoint(sim(jSim,iSim).filter(iAgent,:), sim(jSim,iSim).agent(iAgent).s);
                        sim(jSim,iSim).planner(iAgent).input = sim(jSim,iSim).planner(iAgent).actionSet(:,sim(jSim,iSim).planner(iAgent).actIdx);
                        
                        
                        % mutual information-based optimization
                    case 'MI'
                        
                        for iAction = 1 : sim(jSim,iSim).planner(iAgent).actionSetNum
                            
                            % check whether decision has feasibility in terms of geofence
                            state = UpdateAgentState(sim(jSim,iSim).agent(iAgent).s,sim(jSim,iSim).planner(iAgent).actionSet(iAction),sim(jSim,iSim).clock.dt);
                            if (state(1) > sim(jSim,iSim).field.bufferZone(1) && state(1) < sim(jSim,iSim).field.bufferZone(2)) ...
                                    && (state(2) > sim(jSim,iSim).field.bufferZone(3) && state(2) < sim(jSim,iSim).field.bufferZone(4)) % inside geofence
                                
                                %---------------------------------------------------------------------------------------------------------
                                % Mutual Information computation:
                                %
                                % FIVE APPROACHES are implemented
                                %
                                % 1. particle method considers all measurement/communication awareness possibilities.
                                % 2. particle method of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
                                % 3. Gaussian approximation of which communication is sampled by Pco(Z|Y): motivated by Ryan's approach
                                % 4. Gaussian approximation with modified covariance approach: Maicej's approach
                                % 5. Gaussian with all measurement/communication possibilities: exact when the model is Linear/Gaussian
                                %
                                if sim(jSim,iSim).flagComm == 1
                                    [~, pm, ~, ~, ~] = ComputeInformationTracking(iAgent,iAction,iClock,sim(jSim,iSim),'pmSample');
                                else
                                    [pm, ~, ~, ~, ~] = ComputeInformationTracking(iAgent,iAction,iClock,sim(jSim,iSim),'pmAll');
                                end
                                %---------------------------------------------------------------------------------------------------------
                                
                                % store information data: for pmSample approach only
                                sim(jSim,iSim).planner(iAgent).candidate.Hbefore(:,iAction) = sum(pm.Hbefore);
                                sim(jSim,iSim).planner(iAgent).candidate.Hafter(:,iAction) = sum(pm.Hafter);
                                sim(jSim,iSim).planner(iAgent).candidate.I(iAction) = sum(pm.I);
                                %---------------------------------------------------------------------------------------------------------
                                
                            else % out of geofence
                                sim(jSim,iSim).planner(iAgent).candidate.Hbefore(:,iAction) = inf;
                                sim(jSim,iSim).planner(iAgent).candidate.Hafter(:,iAction) = inf;
                                sim(jSim,iSim).planner(iAgent).candidate.I(iAction) = inf;
                            end
                        end
                        
                        % decision making: maximize mutual information
                        
                        % AD-HOC: when the agent is close to the geofence so that all
                        % cost candidates are infinity, then go to the origin.
                        if min(sim(jSim,iSim).planner(iAgent).candidate.I) == inf
                            % mock filterSet.xhat (allocated as origin)
                            filterSet.xhat = [0 0];
                            sim(jSim,iSim).planner(iAgent).actIdx = MoveToPoint(filterSet, sim(jSim,iSim).agent(iAgent).s);
                        else
                            [~,sim(jSim,iSim).planner(iAgent).actIdx] = max(sim(jSim,iSim).planner(iAgent).candidate.I);
                        end
                        sim(jSim,iSim).planner(iAgent).input = sim(jSim,iSim).planner(iAgent).actionSet(:,sim(jSim,iSim).planner(iAgent).actIdx);
                        
                        sim(jSim,iSim).planner(iAgent).I = sim(jSim,iSim).planner(iAgent).candidate.I(sim(jSim,iSim).planner(iAgent).actIdx);
                        sim(jSim,iSim).planner(iAgent).Hbefore = sim(jSim,iSim).planner(iAgent).candidate.Hbefore(:,sim(jSim,iSim).planner(iAgent).actIdx);
                        sim(jSim,iSim).planner(iAgent).Hafter = sim(jSim,iSim).planner(iAgent).candidate.Hafter(:,sim(jSim,iSim).planner(iAgent).actIdx);
                end
                
            end
            
            
            
            
            %-----------------------------------
            % Actual Agent-Target Dynamics and Measurement
            %-----------------------------------
            
            %-----------------------------------
            % target moving
            for iTarget = 1:sim(jSim,iSim).nTarget
                % target dynamics/store data
                sim(jSim,iSim).target(iTarget).x = UpdateTargetState(sim(jSim,iSim).target(iTarget).x,sim(jSim,iSim).target(iTarget).param,sim(jSim,iSim).clock.dt);
                sim(jSim,iSim).target(iTarget).hist.x(:,iClock+1) = sim(jSim,iSim).target(iTarget).x;
                
                % update plot
                if sim(jSim,iSim).flagPlot
                    set(sim(jSim,iSim).target(iTarget).plot.pos,'Xdata',sim(jSim,iSim).target(iTarget).x(1),'Ydata',sim(jSim,iSim).target(iTarget).x(2));
                    set(sim(jSim,iSim).target(iTarget).plot.id,'position',[sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2)]);
                    addpoints(sim(jSim,iSim).target(iTarget).plot.path,sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2));
                end
            end
            %-----------------------------------
            
            %-----------------------------------
            % agent moving
            % agent dynamics/store data
            for iAgent = 1:sim(jSim,iSim).nAgent
                sim(jSim,iSim).agent(iAgent).s = UpdateAgentState(sim(jSim,iSim).agent(iAgent).s,sim(jSim,iSim).planner(iAgent).input(1),sim(jSim,iSim).clock.dt);
                sim(jSim,iSim).agent(iAgent).hist.s(:,iClock+1) = sim(jSim,iSim).agent(iAgent).s;
                
                % update plot
                if sim(jSim,iSim).flagPlot
                    set(sim(jSim,iSim).agent(iAgent).plot.pos,'Xdata',sim(jSim,iSim).agent(iAgent).s(1),'Ydata',sim(jSim,iSim).agent(iAgent).s(2));
                    set(sim(jSim,iSim).agent(iAgent).plot.id,'position',[sim(jSim,iSim).agent(iAgent).s(1),sim(jSim,iSim).agent(iAgent).s(2)]);
                    addpoints(sim(jSim,iSim).agent(iAgent).plot.path,sim(jSim,iSim).agent(iAgent).s(1),sim(jSim,iSim).agent(iAgent).s(2));
                    set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
                        'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
                end
            end
            %-----------------------------------
            
            
            
            %-----------------------------------
            % take measurement
            for iAgent = 1:sim(jSim,iSim).nAgent
                sim(jSim,iSim).sensor(iAgent,1).plot.bDetect = 0;
                for iTarget = 1:sim(jSim,iSim).nTarget
                    sim(jSim,iSim).sensor(iAgent,iTarget).y = ...
                        TakeMeasurement(sim(jSim,iSim).target(iTarget).x,sim(jSim,iSim).agent(iAgent).s,sim(jSim,iSim).sensor(iAgent,iTarget).param,sim(jSim,iSim).flagSensor);
                    sim(jSim,iSim).sensor(iAgent,iTarget).hist.y(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,iTarget).y;
                    
                    if sim(jSim,iSim).sensor(iAgent,iTarget).y == 1
                        sim(jSim,iSim).sensor(iAgent,1).plot.bDetect = 1;
                    end
                end
                sim(jSim,iSim).sensor(iAgent,1).plot.hist.bDetect(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,1).plot.bDetect;
                
                % update plot
                if sim(jSim,iSim).flagPlot
                    switch sim(jSim,iSim).flagSensor
                        case 'PosLinear' % direct state (location) plot
                            %                         sim(jSim,iSim).sensor(iAgent,1).plot.data.x = ;
                            %                         sim(jSim,iSim).sensor(iAgent,1).plot.data.y = ;
                            %                         set(sim(jSim,iSim).sensor(iAgent,1).plot.fov,'Xdata',sim(jSim,iSim).sensor(iAgent,1).plot.data.x,'Ydata',sim(jSim,iSim).sensor(iAgent,1).plot.data.y);
                            %                         sim(jSim,iSim).sensor(iAgent,1).plot.hist.data.x(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,1).plot.data.x';
                            %                         sim(jSim,iSim).sensor(iAgent,1).plot.hist.data.y(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,1).plot.data.y';
                        case 'range_bear'
                            
                        case 'bear'
                            
                        case 'RF'
                            
                        case 'detection' % sensor coverage plot
                            [sim(jSim,iSim).sensor(iAgent,1).plot.data.x,sim(jSim,iSim).sensor(iAgent,1).plot.data.y,~] = ...
                                GetCircleData(sim(jSim,iSim).agent(iAgent).s(1),sim(jSim,iSim).agent(iAgent).s(2),sim(jSim,iSim).sensor(iAgent,1).param.regionRadius);
                            set(sim(jSim,iSim).sensor(iAgent,1).plot.fov,'Xdata',sim(jSim,iSim).sensor(iAgent,1).plot.data.x,'Ydata',sim(jSim,iSim).sensor(iAgent,1).plot.data.y);
                            sim(jSim,iSim).sensor(iAgent,1).plot.hist.data.x(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,1).plot.data.x';
                            sim(jSim,iSim).sensor(iAgent,1).plot.hist.data.y(:,iClock+1) = sim(jSim,iSim).sensor(iAgent,1).plot.data.y';
                            
                            if sim(jSim,iSim).sensor(iAgent,1).plot.bDetect % when the sensor detects at least one of targets
                                set(sim(jSim,iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(jSim,iSim).sensor(iAgent,1).plot.clr.detect);
                            else
                                set(sim(jSim,iSim).sensor(iAgent,1).plot.fov,'FaceColor',sim(jSim,iSim).sensor(iAgent,1).plot.clr.noDetect);
                            end
                    end
                end
            end
            %-----------------------------------
            
            
            %-----------------------------------
            % particle measurement and agent state sharing through communication
            for iAgent = 1:sim(jSim,iSim).nAgent
                [sim(jSim,iSim).comm(iAgent).beta,sim(jSim,iSim).comm(iAgent).bConnect,sim(jSim,iSim).planner(iAgent).param.agent,sim(jSim,iSim).comm(iAgent).z] = ...
                    ShareInformation(sim(jSim,iSim).agent,sim(jSim,iSim).sensor,sim(jSim,iSim).planner(iAgent).param.agent,sim(jSim,iSim).filter(iAgent).id(1), sim(jSim,iSim).flagActComm);
                sim(jSim,iSim).comm(iAgent).hist.beta(:,iClock+1) = sim(jSim,iSim).comm(iAgent).beta;
                sim(jSim,iSim).comm(iAgent).hist.bConnect(:,iClock+1) = sim(jSim,iSim).comm(iAgent).bConnect;
                sim(jSim,iSim).comm(iAgent).hist.Z(:,:,:,iClock+1) = sim(jSim,iSim).comm(iAgent).z;
            end
            %-----------------------------------
            
            %-----------------------------------
            % Actual measurement and estimation: PF
            %
            % PF is locally performend, and measurement information is delivered
            % under the communication probability
            for iAgent = 1:sim(jSim,iSim).nAgent
                for iTarget = 1:sim(jSim,iSim).nTarget
                    % particle state update
                    sim(jSim,iSim).filter(iAgent,iTarget).pt = UpdateParticle(sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).filter(iAgent,iTarget).param,sim(jSim,iSim).clock.dt);
                    
                    % particle weight update
                    sim(jSim,iSim).filter(iAgent,iTarget).w = ...
                        UpdateParticleWeight(squeeze(sim(jSim,iSim).comm(iAgent).z(:,iTarget,:))',sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).planner(iAgent).param.agent,sim(jSim,iSim).sensor(iAgent).param,sim(jSim,iSim).flagSensor);
                    
                    % compute actual entropy for comparison
                    targetUpdatePdf = ComputePDFMixture(sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).filter(iAgent,iTarget).w,sim(jSim,iSim).planner(iAgent).param,sim(jSim,iSim).flagPdfCompute);
                    sim(jSim,iSim).filter(iAgent,iTarget).Hbefore = ComputeEntropy(targetUpdatePdf,sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).planner(iAgent).param,sim(jSim,iSim).flagPdfCompute);
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.Hbefore(:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).Hbefore;
                    
                    
                    % update plot
                    % AGENT 1 ONLY VISUALIZES PARTICLE INFO BECAUSE OF HUGE
                    % PLOTTING SPACE!
                    if (sim(jSim,iSim).flagPlot) && (iAgent == 1)
                        figure(1+iAgent)
                        subplot(sim(jSim,iSim).filter(iAgent,iTarget).plot.location.col,...
                            sim(jSim,iSim).filter(iAgent,iTarget).plot.location.row,...
                            sim(jSim,iSim).filter(iAgent,iTarget).plot.location.num)
                        set(sim(jSim,iSim).filter(iAgent,iTarget).plot.targetPos,'Xdata',sim(jSim,iSim).target(iTarget).x(1),'Ydata',sim(jSim,iSim).target(iTarget).x(2));
                        set(sim(jSim,iSim).filter(iAgent,iTarget).plot.pt,'Xdata',sim(jSim,iSim).filter(iAgent,iTarget).pt(1,:),'Ydata',sim(jSim,iSim).filter(iAgent,iTarget).pt(2,:),...
                            'Cdata',sim(jSim,iSim).filter(iAgent,iTarget).w);
                        set(sim(jSim,iSim).filter(iAgent,iTarget).plot.targetId,'position',...
                            [sim(jSim,iSim).target(iTarget).x(1),sim(jSim,iSim).target(iTarget).x(2)]);
                        set(gca,'xlim',[sim(jSim,iSim).field.boundary(1),sim(jSim,iSim).field.boundary(2)],...
                            'ylim',[sim(jSim,iSim).field.boundary(3),sim(jSim,iSim).field.boundary(4)])
                    end
                    
                    
                    % resample particle
                    [sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).filter(iAgent,iTarget).w] = ResampleParticle(sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).filter(iAgent,iTarget).w,sim(jSim,iSim).field);
                    
                    % particle filter info update/store
                    sim(jSim,iSim).filter(iAgent,iTarget).xhat = (sim(jSim,iSim).filter(iAgent,iTarget).w*sim(jSim,iSim).filter(iAgent,iTarget).pt')';
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.pt(:,:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).pt;
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.w(:,:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).w;
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.xhat(:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).xhat;
                    
                    
                    % update planner initial info
                    sim(jSim,iSim).planner(iAgent).PTset(iTarget).xhat = sim(jSim,iSim).filter(iAgent,iTarget).xhat;
                    sim(jSim,iSim).planner(iAgent).PTset(iTarget).w = sim(jSim,iSim).filter(iAgent,iTarget).w;
                    sim(jSim,iSim).planner(iAgent).PTset(iTarget).pt = sim(jSim,iSim).filter(iAgent,iTarget).pt;
                    
                    % store optimized infomation data
                    sim(jSim,iSim).planner(iAgent).hist.actIdx(iClock+1) = sim(jSim,iSim).planner(iAgent).actIdx;
                    sim(jSim,iSim).planner(iAgent).hist.input(:,iClock+1) = sim(jSim,iSim).planner(iAgent).input;
                    
                    % compute actual entropy for comparison
                    targetUpdatePdf = ComputePDFMixture(sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).filter(iAgent,iTarget).w,sim(jSim,iSim).planner(iAgent).param,sim(jSim,iSim).flagPdfCompute);
                    sim(jSim,iSim).filter(iAgent,iTarget).Hafter = ComputeEntropy(targetUpdatePdf,sim(jSim,iSim).filter(iAgent,iTarget).pt,sim(jSim,iSim).planner(iAgent).param,sim(jSim,iSim).flagPdfCompute);
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.Hafter(:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).Hafter;
                    
                    % compute actual entropy reduction for comparison
                    sim(jSim,iSim).filter(iAgent,iTarget).I = sim(jSim,iSim).filter(iAgent,iTarget).Hbefore - sim(jSim,iSim).filter(iAgent,iTarget).Hafter;
                    sim(jSim,iSim).filter(iAgent,iTarget).hist.I(:,iClock+1) = sim(jSim,iSim).filter(iAgent,iTarget).I;
                    
                end
            end
            
            % store entropy-based data for planner: only useful for MI-based planning
            if strcmp(sim(jSim,iSim).flagDM,'MI')
                for iAgent = 1:sim(jSim,iSim).nAgent
                    sim(jSim,iSim).planner(iAgent).hist.I(:,iClock+1) = sim(jSim,iSim).planner(iAgent).I;
                    sim(jSim,iSim).planner(iAgent).hist.Hafter(:,iClock+1) = sim(jSim,iSim).planner(iAgent).Hafter';
                    sim(jSim,iSim).planner(iAgent).hist.Hbefore(:,iClock+1) = sim(jSim,iSim).planner(iAgent).Hbefore';
                end
            end
            %-----------------------------------
            
            
            %-----------------------------------
            % take decision making for agent input
            for iAgent = 1:sim(jSim,iSim).nAgent
                sim(jSim,iSim).agent(iAgent).vel = sim(jSim,iSim).planner(iAgent).input(1);
            end
            %-----------------------------------
            
            
            % clock update
            sim(jSim,iSim).clock.hist.time(:,iClock+1) = iClock*sim(jSim,iSim).clock.dt;
            
            % update figure
            if sim(jSim,iSim).flagPlot
                drawnow;
                figure(1)
                title(sprintf('t = %.1f [sec]', sim(jSim,iSim).clock.hist.time(:,iClock+1)))
            end
            
        end
        
        
        % display current simulation number
        fprintf('iSim = %d\n',iSim);
        
        
    end
    
    fprintf('===========\n');
    
end

% result plot based on Monte-Carlo runs
switch flagCondition
    case 'commAware'
        PlotCommAwarePlanningResults(sim);
    case 'planner'
        PlotPlanningResults(sim,'planner');
    case 'nA'
        PlotPlanningResults(sim,'nA');
end

if sim(jSim,iSim).flagLog
    close all;
    save(['a',num2str(sim(1).nAgent),'t',num2str(sim(1).nTarget),'.mat']);
end

