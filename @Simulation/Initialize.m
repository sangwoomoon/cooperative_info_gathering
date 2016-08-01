function [ AGENT, TARGET, ENV ] = Initialize( obj, agentParam, targetParam, landmarkParam, centEstiParam, locEstiParam )
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
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+11},...
        agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+12});
    
    state = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+3}; state(regexp(state,'[[,]]')) = [];
    Q = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+4}; Q(regexp(Q,'[[,]]')) = [];
    bias = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+9}; bias(regexp(bias,'[[,]]')) = [];
    R = agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+10}; R(regexp(R,'[[,]]')) = [];
    
    AGENT(iAgent).DYNAMICS.Initialize(str2double(strsplit(state,';')'));
    AGENT(iAgent).DYNAMICS.SetParameters(diag(str2double(strsplit(Q,';')')),...
        str2double(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+5}),...
        str2double(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+6}));
    
    AGENT(iAgent).SENSOR.Initialize(AGENT(iAgent).id);
    AGENT(iAgent).SENSOR.SetParameters(agentParam{2}{length(agentParam{2})/obj.nAgent*(iAgent-1)+8},...
        str2double(strsplit(bias,';')'),diag(str2double(strsplit(R,';')'))); % Track Object / bias / R
end


%--- Network class initialization ----
obj.NETWORK.Initialize(obj.nAgent,20);
obj.NETWORK.SetParameter();


%--- Individaul KF subclass initialization ----
for iAgent = 1 : obj.nAgent
    
    x_bias = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+9}; x_bias(regexp(x_bias,'[[,]]')) = [];
    Q_bias = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+10}; Q_bias(regexp(Q_bias,'[[,]]')) = [];
    x_target1 = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+3}; x_target1(regexp(x_target1,'[[,]]')) = [];
    Q_target1 = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+4}; Q_target1(regexp(Q_target1,'[[,]]')) = [];
    x_target2 = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+6}; x_target2(regexp(x_target2,'[[,]]')) = [];
    Q_target2 = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+7}; Q_target2(regexp(Q_target2,'[[,]]')) = [];
    R = locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+11}; R(regexp(R,'[[,]]')) = [];
    
    AGENT(iAgent).ESTIMATOR.Initialize(...
        [locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+2};locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+5}],...
        locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+8},...
        {str2double(strsplit(x_bias,';')');str2double(strsplit(x_target1,';')');str2double(strsplit(x_target2,';')')},...
        str2double(locEstiParam{2}{length(locEstiParam{2})/obj.nAgent*(iAgent-1)+12})*eye(10),...
        {diag(str2double(strsplit(Q_bias,';')'));diag(str2double(strsplit(Q_target1,';')'));diag(str2double(strsplit(Q_target2,';')'))},...
        {diag(str2double(strsplit(R,';')'))});
    AGENT(iAgent).ESTIMATOR.SetParameter('fusion',AGENT(iAgent).id);
end

%--- Centralized KF subclass initialization ----

x_a1bias = centEstiParam{2}{8}; x_a1bias(regexp(x_a1bias,'[[,]]')) = [];
Q_a1bias = centEstiParam{2}{9}; Q_a1bias(regexp(Q_a1bias,'[[,]]')) = [];
x_a2bias = centEstiParam{2}{12}; x_a2bias(regexp(x_a2bias,'[[,]]')) = [];
Q_a2bias = centEstiParam{2}{13}; Q_a2bias(regexp(Q_a2bias,'[[,]]')) = [];
x_a3bias = centEstiParam{2}{16}; x_a3bias(regexp(x_a3bias,'[[,]]')) = [];
Q_a3bias = centEstiParam{2}{17}; Q_a3bias(regexp(Q_a3bias,'[[,]]')) = [];
x_target1 = centEstiParam{2}{2}; x_target1(regexp(x_target1,'[[,]]')) = [];
Q_target1 = centEstiParam{2}{3}; Q_target1(regexp(Q_target1,'[[,]]')) = [];
x_target2 = centEstiParam{2}{5}; x_target2(regexp(x_target2,'[[,]]')) = [];
Q_target2 = centEstiParam{2}{6}; Q_target2(regexp(Q_target2,'[[,]]')) = [];
R_a1 = centEstiParam{2}{10}; R_a1(regexp(R_a1,'[[,]]')) = [];
R_a2 = centEstiParam{2}{14}; R_a2(regexp(R_a2,'[[,]]')) = [];
R_a3 = centEstiParam{2}{18}; R_a3(regexp(R_a3,'[[,]]')) = [];

obj.ESTIMATOR.Initialize([centEstiParam{2}{1};centEstiParam{2}{4}],[centEstiParam{2}{7};centEstiParam{2}{11};centEstiParam{2}{15}],...
    {str2double(strsplit(x_a1bias,';')');str2double(strsplit(x_a2bias,';')');str2double(strsplit(x_a3bias,';')');str2double(strsplit(x_target1,';')');str2double(strsplit(x_target2,';')')},...
    str2double(centEstiParam{2}{19})*eye(14),...
    {diag(str2double(strsplit(Q_a1bias,';')'));diag(str2double(strsplit(Q_a2bias,';')'));diag(str2double(strsplit(Q_a3bias,';')'));diag(str2double(strsplit(Q_target1,';')'));diag(str2double(strsplit(Q_target2,';')'))},...
    {diag(str2double(strsplit(R_a1,';')'));diag(str2double(strsplit(R_a2,';')'));diag(str2double(strsplit(R_a3,';')'))});
obj.ESTIMATOR.SetParameter('central',[]);

%--- Data Fusion subclass initialization ----
for iAgent = 1 : obj.nAgent

    xhat = AGENT(iAgent).ESTIMATOR.xhat;
    Phat = AGENT(iAgent).ESTIMATOR.Phat;
    nState = length(xhat) - AGENT(iAgent).SENSOR.nState; % total state num - sensor state num (not available in fusion process)
    
    AGENT(iAgent).FUSION.Initialize(xhat,Phat,nState,AGENT(iAgent).id,obj.nAgent);
    
end



end

