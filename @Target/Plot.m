function o = Plot (o)

plot(o.hist.x(1,:),o.hist.x(3,:),o.plot.color); hold on;
plot(o.hist.x(1,1),o.hist.x(3,1),o.plot.marker(1,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
plot(o.hist.x(1,end),o.hist.x(3,end),o.plot.marker(2,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;

end