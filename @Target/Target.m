classdef Target < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id      % Target id (integers)
        
        DYNAMICS % Dynamics class (element of target, superclass of Linear/Dubins/LinearBias)
        CONTROL  % Control class (set as zero control input in target class now)
        
        plot % plotting options (for state)
        
    end % Properties
    
    methods
        function obj = Target( targetID, option )
            obj = Declare( obj, targetID, option );
        end
        
    end
    
end



