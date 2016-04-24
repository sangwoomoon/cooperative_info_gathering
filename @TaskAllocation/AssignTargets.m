function o = AssignTargets( o, AGENT, TARGET, SIMULATION )
%ASSIGNTARGETS Summary of this function goes here
%   Detailed explanation goes here

    % check what targets are inside of voronoi cell
    for iAgent = 1 : SIMULATION.nAgent
        for iTarget = 1 : SIMULATION.nTarget
            AGENT(iAgent).TA.bTasklist(iTarget) = inpolygon(TARGET(iTarget).x(1),TARGET(iTarget).x(3),...
                SIMULATION.VORONOI.vertices(SIMULATION.VORONOI.cell{iAgent},1),...
                SIMULATION.VORONOI.vertices(SIMULATION.VORONOI.cell{iAgent},2));
        end
        AGENT(iAgent).TA.nSearch = sum(AGENT(iAgent).TA.bTasklist);
    end

end

