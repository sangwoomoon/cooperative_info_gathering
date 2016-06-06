classdef Target < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id      % Target id (integers)
        
        DYNAMICS % Dynamics class (element of target, superclass of Linear/Dubins/LinearBias)
        CONTROL  % Control class (set as zero control input in target class now)
        
        plot % plotting options (for state)
        
    end % Properties
    
    methods
        function o = Target( ENVIRONMENT, iTarget, option )
            o = Default( o, ENVIRONMENT, iTarget, option );
        end
        
    end
    
end



