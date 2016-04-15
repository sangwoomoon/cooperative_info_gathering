function o = Plot(o)
%PLOT Summary of this function goes here
%   Detailed explanation goes here

hold on;

for iAgent = 1 : length(o.C(1,:))
    plot(squeeze(o.hist.C(1,iAgent,2:end)),squeeze(o.hist.C(2,iAgent,2:end)),'color',o.plot.statecolor); hold on;
    plot(o.hist.C(1,iAgent,2),o.hist.C(2,iAgent,2),o.plot.marker(1,:),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on; 
    plot(o.hist.C(1,iAgent,end),o.hist.C(2,iAgent,end),o.plot.marker(2,:),'color',o.plot.statecolor,'MarkerSize',o.plot.markersize,'LineWidth',o.plot.linewidth); hold on;
end

end

