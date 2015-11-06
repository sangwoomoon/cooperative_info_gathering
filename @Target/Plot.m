function o = Plot (o)

plot(o.hist.x(5,:),o.hist.x(7,:),o.plot.color); hold on;
plot(o.hist.x(5,1),o.hist.x(7,1),o.plot.marker(1,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
plot(o.hist.x(5,end),o.hist.x(7,end),o.plot.marker(2,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;

end