function obj = Plot ( obj )

switch (obj.spec)
    case ('Linear')
        plot(obj.hist.x(1,:),obj.hist.x(3,:),obj.plot.line,'color',obj.plot.statecolor); hold on;
        plot(obj.hist.x(1,1),obj.hist.x(3,1),obj.plot.marker(1),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
        plot(obj.hist.x(1,end),obj.hist.x(3,end),obj.plot.marker(2),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
    case ('LinearBias')
        plot(obj.hist.x(3,:),obj.hist.x(5,:),obj.plot.line,'color',obj.plot.statecolor); hold on;
        plot(obj.hist.x(3,1),obj.hist.x(5,1),obj.plot.marker(1),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
        plot(obj.hist.x(3,end),obj.hist.x(5,end),obj.plot.marker(2),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
    case ('Dubins')
        plot(obj.hist.x(1,:),obj.hist.x(2,:),obj.plot.line,'color',obj.plot.statecolor); hold on;
        plot(obj.hist.x(1,1),obj.hist.x(2,1),obj.plot.marker(1),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
        plot(obj.hist.x(1,end),obj.hist.x(2,end),obj.plot.marker(2),'color',obj.plot.statecolor,'MarkerSize',obj.plot.markersize,'LineWidth',obj.plot.linewidth); hold on;
end

end