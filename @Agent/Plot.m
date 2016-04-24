function o = Plot ( o )

plot(o.hist.s(1,:),o.hist.s(3,:),o.plot.line,'MarkerSize',0.2); hold on;
plot(o.hist.s(1,1),o.hist.s(3,1),o.plot.marker(1),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
plot(o.hist.s(1,end),o.hist.s(3,end),o.plot.marker(2),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
text(o.hist.s(1,end)-10,o.hist.s(3,end)-10,num2str(o.id)); hold on;

end