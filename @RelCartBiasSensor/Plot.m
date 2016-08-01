function obj = Plot ( obj, ENVIRONMENT )

%%Plot cartesian relative target measurements:
for iMeasure = 1 : length(obj.hist.meas)
    for iTarget = 1 : length(obj.hist.meas{iMeasure})
        switch (obj.hist.meas{iMeasure}(iTarget).id)
            case ('LANDMARK') % when measured object is a landmark
                pos = ENVIRONMENT.LANDMARK.DYNAMICS.GetPosition();
                plot(pos(1) + obj.hist.meas{iMeasure}(iTarget).y(1), pos(2) + obj.hist.meas{iMeasure}(iTarget).y(2),'.','color',obj.plot.color); hold on;
            otherwise % object is a target
                plot(obj.hist.meas{iMeasure}(iTarget).y(1),obj.hist.meas{iMeasure}(iTarget).y(2),'.','color',obj.plot.color); hold on;
        end
    end
end

end