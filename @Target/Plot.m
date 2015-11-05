function o = Plot (o)

plot(o.hist.x(3,:),o.hist.x(5,:),o.plot.color); hold on;
plot(o.hist.x(3,1),o.hist.x(5,1),o.plot.marker(1,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
plot(o.hist.x(3,end),o.hist.x(5,end),o.plot.marker(2,:),'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;

end