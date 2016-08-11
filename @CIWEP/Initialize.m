function obj = Initialize(obj, xhat, Phat, nTargetState, agentID, nAgent)

% set sender determinent array for fusion communication
obj.bSend2others = ones(1,nAgent);
obj.bSend2others(agentID) = 0;

obj.xhat = xhat;
obj.Phat = Phat;

nTotalState = length(xhat);

% store only marginal matrix
obj.hist.xhatMgn = nan(nTargetState,1);
obj.hist.PhatMgn = nan(nTargetState,nTargetState,1);

obj.hist.xhat = nan(nTotalState,1);
obj.hist.Phat = nan(nTotalState,nTotalState,1);

obj.hist.omega = nan(nAgent,2,1); 

obj.hist.stamp = 0;
    
end