function stateNext = UpdateTargetState(stateNow,param,dt)
    stateNext = LinModel(stateNow,param);
end