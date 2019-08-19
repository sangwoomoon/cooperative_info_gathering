function F = ComputeStateTransitionMatrix(targetState,property)

switch property
    case 'Pos'
        F = eye(length(targetState));
    case 'PosVel'
        
    case 'PosRF'
        F = eye(length(targetState));
        
end

