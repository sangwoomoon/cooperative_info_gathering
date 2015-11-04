function o = Plot (o)

plot(o.hist.x(1,:),o.hist.x(2,:),o.plot.color); hold on;
plot(o.hist.x(1,1),o.hist.x(2,1),o.plot.marker(1,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
plot(o.hist.x(1,end),o.hist.x(2,end),o.plot.marker(2,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;

end