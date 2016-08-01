function Z = CreatePackage( obj, agentID, position, measurement, input, Phat, xhat )
%CREATEPACKAGE creates package to send another agent for fusion process 

    for iAgent = 1 : length(obj.bSend2others)
        if obj.bSend2others(iAgent) == 1 % when the agent sends it to iAgent-th agent
            Z{iAgent}.senderID = agentID; % sender id (for indexing)
            Z{iAgent}.receiverID = iAgent; % reciver id (for indexing)
            
            % add position of agent
            Z{iAgent}.pos = position;
            
            % add measurement data
            Z{iAgent}.meas = measurement;
            
            % add input data
            Z{iAgent}.u = input;
            
            % add estimates
            Z{iAgent}.Phat = Phat; % beware of that it is combined by bias states
            Z{iAgent}.xhat = xhat; % beware of that without bias (ignore 5,6th element)
        else
            Z{iAgent} = [];%  nullify all parts on that element
        end
    end


end

