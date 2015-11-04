classdef Clock 
    properties   ( SetAccess = public, GetAccess = public )
        
        t0 % initial time
        dt % discrete time step
        delt % discrete time step for each procedure
        nt % number of time steps
        tf % termination time
        
        ct % current time
        
        tvec % time vector
        
    end % Properties
    

    methods
        function o = Clock()
           o = Default(o);
        end
        
        o = get( o, varargin );
    end
    
end



