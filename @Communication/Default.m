function o = Default (o, SIMULATION, CLOCK)
    o.C = ones(SIMULATION.nAgent)-eye(SIMULATION.nAgent); % perfect communication
end