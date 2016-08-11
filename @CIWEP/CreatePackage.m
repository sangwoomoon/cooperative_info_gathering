function Z = CreatePackage( obj, agentID, position, bTrackTarget, measurement, input, Phat, xhat )
%CREATEPACKAGE creates package to send another agent for fusion process 

nTotalState = length(xhat);
nTargetState = length(obj.hist.xhatMgn(:,1,1));
nBiasState = nTotalState-nTargetState;

    for iAgent = 1 : length(obj.bSend2others)
        if obj.bSend2others(iAgent) == 1 % when the agent sends it to iAgent-th agent
            Z{iAgent}.senderID = agentID; % sender id (for indexing)
            Z{iAgent}.receiverID = iAgent; % reciver id (for indexing)
            
            % add position of agent
            Z{iAgent}.pos = position;
            
            % add binary for tracking targets (will be separated later)
            Z{iAgent}.bTrack = bTrackTarget;
            
            % add measurement data
            Z{iAgent}.meas = measurement;
            
            % add input data
            Z{iAgent}.u = input;
            
            % add estimates
            Z{iAgent}.Phat = Phat(nBiasState+1:end,nBiasState+1:end); % only for target info
            Z{iAgent}.xhat = xhat(nBiasState+1:end); % only for target info
            
        else
            
            Z{iAgent} = [];%  nullify all parts on that element
        end
        
    end


end

