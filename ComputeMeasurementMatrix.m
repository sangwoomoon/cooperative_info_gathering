function H = ComputeMeasurementMatrix(targetState,agentState,property)

switch property
    case 'PosLinear'
        H = eye(length(targetState));
    case 'range_bear'
        H = ComputeRangeBearMeasMatrix(agentState,targetState);
    case 'bear'
        H = ComputeBearMeasMatrix(agentState,targetState);
end

