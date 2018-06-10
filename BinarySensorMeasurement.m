function meas = BinarySensorMeasurement(targetState,agentState,sensorParam)

    if IsInCircleBasedRegion(targetState,agentState,sensorParam.regionRadius)
        meas = binornd(1,sensorParam.detectBeta);
    else
        meas = 0;
    end
    
end
