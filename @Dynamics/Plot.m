function o = Plot ( o )

switch (o.spec)
    case ('Linear')
        plot(o.hist.s(1,:),o.hist.s(3,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.s(1,1),o.hist.s(3,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.s(1,end),o.hist.s(3,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
    case ('LinearBias')
        plot(o.hist.s(3,:),o.hist.s(5,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.s(3,1),o.hist.s(5,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.s(3,end),o.hist.s(5,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
    case ('Dubins')
        plot(o.hist.s(1,:),o.hist.s(2,:),o.plot.line,'color',o.plot.statecolor); hold on;
        plot(o.hist.s(1,1),o.hist.s(2,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
        plot(o.hist.s(1,end),o.hist.s(2,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
end

end