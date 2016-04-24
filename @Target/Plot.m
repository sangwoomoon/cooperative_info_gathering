function o = Plot (o)

plot(o.hist.x(1,:),o.hist.x(3,:),'color',o.plot.statecolor); hold on;
plot(o.hist.x(1,1),o.hist.x(3,1),o.plot.marker(1,:),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
plot(o.hist.x(1,end),o.hist.x(3,end),o.plot.marker(2,:),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
text(o.hist.x(1,end)-10,o.hist.x(3,end)-10,num2str(o.id)); hold on;

end