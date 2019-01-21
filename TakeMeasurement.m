function meas = TakeMeasurement(targetState,agentState,sensorParam,property)

switch property
    case 'PosLinear'
        meas = PositionMeasurement(targetState,sensorParam);
    case 'range_bear'
        
    case 'detection'
        meas = BinarySensorMeasurement(targetState,agentState,sensorParam);
end

end