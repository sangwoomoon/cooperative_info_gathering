function o = CheckTerminateSim( o, AGENT )
%CHECKTERMINATESIM Summary of this function goes here
%   Detailed explanation goes here

chkDistribSim = 0;
for iAgent = 1 : o.nAgent
    if AGENT(iAgent).actIdx == 5
        chkDistribSim = chkDistribSim + 1;
    end
end

if o.bCentral == 1
    chkCentralSim = 0;
    
    if sum(o.DM.actIdx) == 5*o.nAgent
        chkCentralSim = 1;
    end
    
    if chkDistribSim == o.nAgent && chkCentralSim == 1
        o.bSim = 0;
    end
else
    if chkDistribSim == o.nAgent
        o.bSim = 0;
    end
end

end

