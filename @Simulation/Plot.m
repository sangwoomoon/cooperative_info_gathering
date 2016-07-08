function o = Plot(o, AGENT, CLOCK)

CLOCK.Plot();

% Agent plot (FIGURE 1)
for iAgent = 1 : o.nAgent
    AGENT(iAgent).Plot();
end

% Central DM plot
o.DM.Plot();

drawnow;

end
