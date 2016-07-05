function o = Plot ( o )

plot(o.hist.s(1,:),o.hist.s(2,:),o.plot.line,'LineWidth',0.5); hold on;
plot(o.hist.s(1,1),o.hist.s(2,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
plot(o.hist.s(1,end),o.hist.s(2,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;

end