function o = DetermineCommStatus( o, SIMULATION, AGENT )
%DETERMINECOMMSTATUS function determines the network graph between agents
%   it is now based on the distance between agents

    for ii = 1 : SIMULATION.nAgent
        for jj = 1 : SIMULATION.nAgent
            if ii == jj
                o.graph(ii,jj) = 0; % not to communicate itself
            else
                if distance(AGENT(ii).DYNAMICS.s,AGENT(jj).DYNAMICS.s) < o.range
                    o.graph(ii,jj) = 1; % within range, they can communicate
                else
                    o.graph(ii,jj) = 0; % out of range
                end
            end
        end
    end

end


function dist = distance(a,b)
    dist = sqrt((a(1)-b(1))^2+(a(3)-b(3))^2);
end

