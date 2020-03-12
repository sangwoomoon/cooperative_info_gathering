function stateNext = QuadrotorModel(stateNow,input,dt)
    
    switch input
        case 'N' % move North
            u = [0;1;0;0];
        case 'E' % move East
            u = [1;0;0;0];
        case 'W' % move West
            u = [-1;0;0;0];
        case 'S' % move South
            u = [0;-1;0;0];
        case 'T' % stop moving
            u = [0;0;0;0];
    end    
    
    stateNext = stateNow + u*dt;
end