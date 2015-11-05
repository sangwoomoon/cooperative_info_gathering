function o = Plot ( o ,AGENT )

for iTarget = 1 : length(o.Rt) % for number of targets,
    %%Plot cartesian relative target measurements:
    plot(o.hist.y(2*(iTarget-1)+1,:),o.hist.y(2*iTarget,:),'.','color',o.plot.reltargetcolor); hold on;
end

end