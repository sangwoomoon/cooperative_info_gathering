function stateNext = UpdateAgentState(stateNow,input,dt)
    stateNext = UniCycleModel(stateNow,input,dt);
end