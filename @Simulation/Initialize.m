function [ AGENT, TARGET, ENV ] = Initialize( obj, agentParam, targetParam, landmarkParam, centEstiParam, locEstiParam, networkParam )
%INITIALIZE initializes all classes for simulation with imported data from
%text files

%--- Environment class and its landmarks initialization ----
ENV = Environment();

for iLandmark = 1 : obj.nLandmark
    state = landmarkParam{2}{length(landmarkParam{2})/obj.nLandmark*(iLandmark-1)+3}; state(regexp(state,'[[,]]')) = [];
    
    ENV.LANDMARK = Landmark(landmarkParam{2}{length(landmarkParam{2})/obj.nLandmark*(iLandmark-1)+1}, landmarkParam{2}{length(landmarkParam{2})/obj.nLandmark*(iLandmark-1)+2});
    ENV.LANDMARK.DYNAMICS.Initialize(str2double(strsplit(state,';')'));
    ENV.LANDMARK.DYNAMICS.SetParameters([],[],[]); % parameter setting order : bKFx, Q, RelTol, AbsTol (last two is for ODE45)
end

%--- Target class initialization ----
for iTarget = 1 : obj.nTarget
    TARGET(iTarget) = Target(str2num(targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+1}), targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+2});
    
    state = targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+3}; state(regexp(state,'[[,]]')) = [];
    Q = targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+4}; Q(regexp(Q,'[[,]]')) = [];
    
    TARGET(iTarget).DYNAMICS.Initialize(str2double(strsplit(state,';')'));
    TARGET(iTarget).DYNAMICS.SetParameters(diag(str2double(strsplit(Q,';'))'),...
        str2double(targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+5}),...
        str2double(targetParam{2}{length(targetParam{2})/obj.nTarget*(iTarget-1)+6}));
end

%--- Agent class initialization ----
for iAgent = 1 : obj.nAgent
    AGENT(iAgent) = Agent();
    AGENT(iAgent).Initialize(str2num(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+1}),...
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+2},...
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+7},...
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+12},...
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+13});
    
    % load and convert bTrackingTarget as double-array
    bTrackingTarget = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+9}; bTrackingTarget(regexp(bTrackingTarget,'[[,]]')) = [];
    bTrackingTarget = str2double(strsplit(bTrackingTarget,';'));
    
    state = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+3}; state(regexp(state,'[[,]]')) = [];
    Q = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+4}; Q(regexp(Q,'[[,]]')) = [];
    bias = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+10}; bias(regexp(bias,'[[,]]')) = [];
    R = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+11}; R(regexp(R,'[[,]]')) = [];
    
    AGENT(iAgent).DYNAMICS.Initialize(str2double(strsplit(state,';')'));
    AGENT(iAgent).DYNAMICS.SetParameters(diag(str2double(strsplit(Q,';')')),...
        str2double(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+5}),...
        str2double(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+6}));
    
    AGENT(iAgent).SENSOR.Initialize(AGENT(iAgent).id);
    switch (agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+7})
        case ('InertCart')
            AGENT(iAgent).SENSOR.SetParameters(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+8},bTrackingTarget,...
                diag(str2double(strsplit(R,';')'))); % Track Object / bTrackingTarget / R            
        case ('RelCart')
            AGENT(iAgent).SENSOR.SetParameters(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+8},bTrackingTarget,...
                str2double(strsplit(bias,';')'),diag(str2double(strsplit(R,';')'))); % Track Object / bTrackingTarget / bias = 0 (since the estimator may consider bias) / R
        case ('RelCartBias')
            AGENT(iAgent).SENSOR.SetParameters(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+8},bTrackingTarget,...
                str2double(strsplit(bias,';')'),diag(str2double(strsplit(Q,';')')),diag(str2double(strsplit(R,';')'))); % Track Object / bTrackingTarget / bias / Q / R
    end
end


%--- Network class initialization ----
range = str2double(networkParam{2}{2});
obj.NETWORK.Initialize(obj.nAgent,range);
obj.NETWORK.SetParameter();


%--- Individaul KF subclass initialization ----
for iAgent = 1 : obj.nAgent
    
    Q = [];
    R = [];
    xhat_0 = [];
    targetSpec = [];
    sensorSpec = [];
    
    % load and convert guessed bias info as double matrix/vector
    xhat_0{1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3+obj.nTarget*3}; 
    xhat_0{1}(regexp(xhat_0{1},'[[,]]')) = [];
    
    Q{1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3+obj.nTarget*3+1};
    Q{1}(regexp(Q{1},'[[,]]')) = [];

    if ~isempty(xhat_0{1}) % when the bias is not exist (without bias)
        xhat_0{1} = str2double(strsplit(xhat_0{1},';')');
        Q{1} = diag(str2double(strsplit(Q{1},';')'));
        nState = length(xhat_0{1});
    else
        xhat_0{1} = []; % nullify all bias part
        Q{1} = []; % nullify all bias part
        nState = 0;
    end
    
    % load guessed sensor info
    sensorSpec{1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+obj.nTarget*3+2}; % cell type :: for centralized Estimation
    R = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+obj.nTarget*3+5}; R(regexp(R,'[[,]]')) = [];
    R = {diag(str2double(strsplit(R,';')'))}; % cell type :: for centralized Estimation
    
    % load and convert guessed target info as double matrix/vector
    for iTarget = 1 : obj.nTarget
        if (AGENT(iAgent).SENSOR.bTrack(iTarget) == 1)
            targetSpec{end+1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3*(iTarget-1)+2};
            
            xhat_0{end+1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3*(iTarget-1)+3}; xhat_0{end}(regexp(xhat_0{end},'[[,]]')) = [];
            xhat_0{end} = str2double(strsplit(xhat_0{end},';')');
            nState = nState + length(xhat_0{end});
            
            Q{end+1} = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3*(iTarget-1)+4}; Q{end}(regexp(Q{end},'[[,]]')) = [];
            Q{end} = diag(str2double(strsplit(Q{end},';')'));
        end
    end
    
    % load and convert initial Phat as double matrix
    iniVariance = str2double(locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+obj.nTarget*3+6});
    Phat_0 = iniVariance*eye(nState);
    
    for iEsti = 1 : 2 % local process vs. local process with fusion (for comparison)
        AGENT(iAgent).ESTIMATOR(iEsti).Initialize(AGENT(iAgent).SENSOR.bTrack,targetSpec,sensorSpec,xhat_0,Phat_0,Q,R);
        AGENT(iAgent).ESTIMATOR(iEsti).SetParameter('fusion',AGENT(iAgent).id);
    end
    
end

%--- Centralized KF subclass initialization ----

Q = [];
R = [];
xhat_0 = [];
targetSpec = [];
sensorSpec = [];

nState = 0;
bTrack = [];

% gather guessed agant's sensor bias info (even if it is not assigned by users!) and its measurement cov matrix
for iAgent = 1 : obj.nAgent
    
    sensorSpec{iAgent} = centEstiParam{2}{obj.nTarget*3+1+4*(iAgent-1)};
    bTrack = [bTrack;AGENT(iAgent).SENSOR.bTrack];
    
    xhat_0{iAgent} = centEstiParam{2}{obj.nTarget*3+2+4*(iAgent-1)}; xhat_0{iAgent}(regexp(xhat_0{iAgent},'[[,]]')) = [];
    Q{iAgent} = centEstiParam{2}{obj.nTarget*3+3+4*(iAgent-1)}; Q{iAgent}(regexp(Q{iAgent},'[[,]]')) = [];
    
    if ~isempty(xhat_0{iAgent})
        xhat_0{iAgent} = str2double(strsplit(xhat_0{iAgent},';')');
        Q{iAgent} = diag(str2double(strsplit(Q{iAgent},';')'));
        nState = nState + length(xhat_0{iAgent});
    else % if no bias part
        xhat_0{iAgent} = [];
        Q{iAgent} = [];
    end
    
    R{iAgent} = centEstiParam{2}{obj.nTarget*3+4+4*(iAgent-1)}; R{iAgent}(regexp(R{iAgent},'[[,]]')) = [];
    R{iAgent} = diag(str2double(strsplit(R{iAgent},';')'));
end

% gather guessed target info
for iTarget = 1 : obj.nTarget
    targetSpec{iTarget} = centEstiParam{2}{1+3*(iTarget-1)};
    
    xhat_0{obj.nAgent+iTarget} = centEstiParam{2}{2+3*(iTarget-1)}; xhat_0{obj.nAgent+iTarget}(regexp(xhat_0{obj.nAgent+iTarget},'[[,]]')) = [];
    xhat_0{obj.nAgent+iTarget} = str2double(strsplit(xhat_0{obj.nAgent+iTarget},';')');
    nState = nState + length(xhat_0{obj.nAgent+iTarget});
    
    Q{obj.nAgent+iTarget} = centEstiParam{2}{3+3*(iTarget-1)}; Q{obj.nAgent+iTarget}(regexp(Q{obj.nAgent+iTarget},'[[,]]')) = [];
    Q{obj.nAgent+iTarget} = diag(str2double(strsplit(Q{obj.nAgent+iTarget},';')'));
end

% load and convert initial Phat as double matrix
iniVariance = str2double(locEstiParam{2}{end});
Phat_0 = iniVariance*eye(nState);

obj.ESTIMATOR.Initialize(bTrack,targetSpec,sensorSpec,xhat_0,Phat_0,Q,R);
obj.ESTIMATOR.SetParameter('central',[]);

%--- Data Fusion subclass initialization ----
for iAgent = 1 : obj.nAgent

    xhat = AGENT(iAgent).ESTIMATOR.xhat;
    Phat = AGENT(iAgent).ESTIMATOR.Phat;
    nState = length(xhat) - AGENT(iAgent).SENSOR.nState; % total state num - sensor state num (not available in fusion process)
    
    AGENT(iAgent).FUSION.Initialize(xhat,Phat,nState,AGENT(iAgent).id,obj.nAgent);
    
end



end

