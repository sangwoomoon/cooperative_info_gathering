function obj = Plot ( obj )

%%Plot cartesian relative target measurements:
for iMeasure = 1 : length(obj.hist.meas)
    for iTarget = 1 : length(obj.hist.meas{iMeasure})
        plot(obj.hist.meas{iMeasure}(iTarget).y(1),obj.hist.meas{iMeasure}(iTarget).y(2),'.','color',obj.plot.color); hold on;
    end
end

end