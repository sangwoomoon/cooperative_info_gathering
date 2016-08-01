classdef Clock < handle
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
        function obj = Clock()
           obj = Declare(obj);
        end
        
        % set properties
        Initialize(obj, initialTime, TimeStep, IterationNumber, FDDFInterval);
    end
    
end



