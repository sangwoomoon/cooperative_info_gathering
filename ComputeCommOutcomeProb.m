
function commOutcomeProb = ComputeCommOutcomeProb(plannerAgent,commStatus,id,flagComm)

nAgent = length(plannerAgent);



% beta initialization
commOutcomeProb = zeros(1,nAgent);


for iAgent = 1:nAgent
    
    % considering communication-aware events
    if flagComm
        
        beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);
        
        if commStatus(iAgent) % when connected
            commOutcomeProb(iAgent) = beta;
        else % when disconnected
            commOutcomeProb(iAgent) = 1 - beta;
        end
        
    % when the communication is separatively considered from the measurements
    % : output is real commProb with H(X|Y).
    elseif flagComm == 2
        
        beta = ComputeCommProb(plannerAgent(id).s,plannerAgent(iAgent).s);

        if commStatus(iAgent) % when connected
            commOutcomeProb(iAgent) = beta;            
        else
            commOutcomeProb(iAgent) = 1 - beta;            
        end
        
    % under perfect communication
    else
        
        commOutcomeProb(iAgent) = 1;
        
    end
    
end

end