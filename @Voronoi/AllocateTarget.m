function o = AllocateTarget( o, AGENT, SIMULATION )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% ASSUME : target location is considered as estimated location

for iAgent = 1 : length(AGENT)
    for iTarget = 1 : length(SIMULATION.nTarget)
        
        x = AGENT(iAgent).LOCAL_KF.hist.XhatGlobal(4*(iTarget-1)+1,end);
        y = AGENT(iAgent).LOCAL_KF.hist.XhatGlobal(4*(iTarget-1)+3,end);
        
        if inpolygon(x,y,o.vertices(o.cell{iAgent},1),o.vertices(o.cell{iAgent},2))
            AGENT(iAgent).LOCAL_KF.bTrack(1,iTarget) = 1;
        end
    end
end

end

