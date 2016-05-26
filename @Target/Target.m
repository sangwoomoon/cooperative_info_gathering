classdef Target < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id      % Target id (integers)
        
        DYNAMICS % Dynamics class (element of target, superclass of Linear/Dubins/LinearBias)
        CONTROL  % Control class (set as zero control input in target class now)
        
    end % Properties
    
    methods
        function o = Target( Clock, ENVIRONMENT, iTarget, option )
            o = Default( o, Clock, ENVIRONMENT, iTarget, option );
        end
        
    end
    
end



