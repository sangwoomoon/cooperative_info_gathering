function stateNext = UpdateAgentState(stateNow,input,dt,flagAgent)
    switch flagAgent
        case 'unicycle'
            stateNext = UniCycleModel(stateNow,input,dt);
        case 'quadrotor'
            stateNext = QuadrotorModel(stateNow,input,dt);
    end
end