function obj = Plot ( obj, PlottedClass )

plot(obj.hist.x(1,:),obj.hist.x(3,:),PlottedClass.plot.line,'color',PlottedClass.plot.statecolor); hold on;
plot(obj.hist.x(1,1),obj.hist.x(3,1),PlottedClass.plot.marker(1),'color',PlottedClass.plot.statecolor,'MarkerSize',PlottedClass.plot.markersize,'LineWidth',PlottedClass.plot.linewidth); hold on;
plot(obj.hist.x(1,end),obj.hist.x(3,end),PlottedClass.plot.marker(2),'color',PlottedClass.plot.statecolor,'MarkerSize',PlottedClass.plot.markersize,'LineWidth',PlottedClass.plot.linewidth); hold on;

end