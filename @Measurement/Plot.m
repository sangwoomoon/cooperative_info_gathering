function o = Plot ( o ,AGENT )

for iTarget = 1 : length(o.Rt) % for number of targets,
    %%Plot cartesian relative target measurements:
    plot(AGENT.hist.s(1,:)+o.hist.y(2*(iTarget-1)+1,:),AGENT.hist.s(3,:)+o.hist.y(2*iTarget,:),'.','color',o.plot.reltargetcolor); hold on;
end

%%Plot absolute platform measurements:
plot(o.hist.y(2*length(o.Rt)+1,:),o.hist.y(2*length(o.Rt)+2,:),'.','color',o.plot.absmeasurecolor); hold on;


end