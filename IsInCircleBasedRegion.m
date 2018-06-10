% sub-function for BinarySensorModel: Circle-based determinent
function bLocate = IsInCircleBasedRegion(targetState,agentState,radius)
    bLocate = norm(targetState(1:2)-agentState(1:2)) < radius;
end