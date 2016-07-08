classdef Clock 
    properties   ( SetAccess = public, GetAccess = public )
        
        t0 % initial time
        
        nt % number of time steps
        tf % termination time
        
        ct % current time
        
        tvec % time vector
        
        plot
        
    end % Properties
    

    methods
        function o = Clock( initialTime, IterationNumber, SIM )
           o = Default(o, initialTime, IterationNumber, SIM );
        end
        
        o = get( o, varargin );
    end
    
end



