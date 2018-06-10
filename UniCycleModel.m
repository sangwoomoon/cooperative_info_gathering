function stateNext = UniCycleModel(stateNow,input,dt)
    psiNext = stateNow(3) + input*dt;
    
    stateNext = ...
        [stateNow(1) + stateNow(4)*cos(psiNext)*dt;
         stateNow(2) + stateNow(4)*sin(psiNext)*dt;
         psiNext;
         stateNow(4)];
end