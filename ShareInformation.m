%-----------------------------------
% Share data package through communication
% data package:
% 1) agent state
%    - when the communication falis, the sender's state is not updated
%    - the 1st statement should be modified when considering Bayesian
%      Learning
% 2) measurement
%    - when the communication fails, the measurement is null and it will
%    not be affected to the filtering process
%-----------------------------------
function [commBeta,bConnect,agentStateSet,Z] = ShareInformation(agent,sensor,plannerAgent,id)

% 0. initialization
nAgent = length(agent);
nTarget = length(sensor(1,:));
commBeta = nan(nAgent,1);
bConnect = nan(nAgent,1);
Z = nan(nAgent,nTarget);
agentStateSet = plannerAgent;

for iAgent = 1:nAgent
    % 1. compute communication probability
    commBeta(iAgent,1) = ComputeCommProb(agent(id).s,agent(iAgent).s);
    
    % 2. compute outcome of package delivery
    bConnect(iAgent,1) = binornd(1,commBeta(iAgent,1));
    
    % 3. update information based on the outcome
    if bConnect(iAgent,1) == 1
       Z(iAgent,:) = sensor(iAgent,:).y;
       agentStateSet(iAgent).s = agent(iAgent).s;
    end
    
end

end