function obj = InitializeState( obj, input_state )
%INITIALIZESTATE Summary of this function goes here
%   Detailed explanation goes here

    % allocate input state by users to x
    obj.x = input_state;
    
    % store history
    obj.hist.x = obj.x;
    obj.hist.x_pred = nan(length(obj.x),1);
    
    obj.hist.stamp = 0; % initial time step (zero IC for time)

end

