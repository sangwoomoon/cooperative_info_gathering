function o = TakeTargetID( o, AGENT, SIMULATION )
%TAKETARGETID Summary of this function goes here
%   Detailed explanation goes here

for iCell = 1 : length(o.cell)
    
    % reset / initialize
    Vx = [];
    Vy = [];
    
    % accumulate vertices
    for iVert = 1 : length(o.cell{iCell})
        Vx = [Vx; o.vertices(o.cell{iCell}(iVert),1)];
        Vy = [Vy; o.vertices(o.cell{iCell}(iVert),2)];
    end
    
    if inpolygon(AGENT.s(1),AGENT.s(3),Vx,Vy) 
        
        for iTarget = 1 : SIMULATION.nTarget
            
            targetPos = [AGENT.LOCAL_KF.Xhat(4*(iTarget-1)+1);...
                         AGENT.LOCAL_KF.Xhat(4*(iTarget-1)+3)]; 
            
            if inpolygon(targetPos(1),targetPos(2),Vx,Vy)
                o.TrackID = iTarget;
            end
            
        end
        
    end
    
end

end

