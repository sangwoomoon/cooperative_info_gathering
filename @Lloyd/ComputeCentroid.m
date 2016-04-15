function o = ComputeCentroid( o, SIMULATION )
%COMPUTECENTROID Summary of this function goes here
%   Detailed explanation goes here

V = SIMULATION.VORONOI.vertices;

for iAgent = 1 : SIMULATION.nAgent
    
    C = SIMULATION.VORONOI.cell{iAgent};
    
    X = V(C,1);
    Y = V(C,2);
    
    o.C(:,iAgent) = [sum(X)/length(C), sum(Y)/length(C)]';
end

o.hist.C(:,:,end+1) = o.C; % store data

end

