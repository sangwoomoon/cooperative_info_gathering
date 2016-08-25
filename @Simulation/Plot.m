function o = Plot(o, AGENT, ENV, CLOCK)

CLOCK.Plot();

% Environment plot (FIGURE 1)
ENV.Plot();

% Agent plot (FIGURE 1)
for iAgent = 1 : o.nAgent
    AGENT(iAgent).Plot();
end

% Central DM plot
if o.bCentral == 1
    o.DM.Plot();
end

axis([ENV.xlength(1) ENV.xlength(2)...
    ENV.ylength(1) ENV.ylength(2)]);
axis equal;
set(gcf,'color','w');

drawnow;

end
