function obj = Plot ( obj, PlottedClass )

plot(obj.hist.x(3,:),obj.hist.x(5,:),PlottedClass.plot.line,'color',PlottedClass.plot.statecolor); hold on;
plot(obj.hist.x(3,1),obj.hist.x(5,1),PlottedClass.plot.marker(1),'color',PlottedClass.plot.statecolor,'MarkerSize',PlottedClass.plot.markersize,'LineWidth',PlottedClass.plot.linewidth); hold on;
plot(obj.hist.x(3,end),obj.hist.x(5,end),PlottedClass.plot.marker(2),'color',PlottedClass.plot.statecolor,'MarkerSize',PlottedClass.plot.markersize,'LineWidth',PlottedClass.plot.linewidth); hold on;

end