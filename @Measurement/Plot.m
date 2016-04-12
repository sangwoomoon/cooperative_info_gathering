function o = Plot ( o ,AGENT )

%%Plot cartesian relative target measurements:
Xmeas = AGENT.hist.s(1,1:end-1)+o.hist.y(1,2:end).*cos(o.hist.y(2,2:end));
Ymeas = AGENT.hist.s(3,1:end-1)+o.hist.y(1,2:end).*sin(o.hist.y(2,2:end));

plot(Xmeas, Ymeas,'.','color',o.plot.reltargetcolor); hold on;

end