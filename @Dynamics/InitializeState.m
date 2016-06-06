function obj = InitializeState( obj, input_state )
%INITIALIZESTATE Summary of this function goes here
%   Detailed explanation goes here

    % allocate input state by users to x
    obj.x = input_state;
    
    % external states are allocated as eqaul to interal one now
    obj.x_e = input_state;
    
    % store history
    obj.hist.x = obj.x;
    obj.hist.stamp = 0; % initial time step (zero IC for time)
    
    obj.hist.x_e = obj.x_e;

end

