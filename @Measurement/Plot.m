function o = Plot ( o ,AGENT )

%%Plot cartesian relative target measurements:
plot(o.hist.y(1,:),o.hist.y(2,:),'.','color',o.plot.reltargetcolor); hold on;

end