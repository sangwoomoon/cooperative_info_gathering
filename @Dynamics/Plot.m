function o = Plot ( o )

switch (o.spec)
    case ('Linear')
        plot(o.hist.x(1,:),o.hist.x(3,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.x(1,1),o.hist.x(3,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.x(1,end),o.hist.x(3,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
    case ('LinearBias')
        plot(o.hist.x(3,:),o.hist.x(5,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.x(3,1),o.hist.x(5,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.x(3,end),o.hist.x(5,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
    case ('Dubins')
        plot(o.hist.x(1,:),o.hist.x(2,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.x(1,1),o.hist.x(2,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.x(1,end),o.hist.x(2,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
end

end