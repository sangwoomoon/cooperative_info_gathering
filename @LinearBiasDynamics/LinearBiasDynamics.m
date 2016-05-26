classdef LinearBiasDynamics < Dynamics
    %LinearBiasDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        function o = LinearBiasDynamics( CONTROL, CLOCK, id )
            o@Dynamics(CONTROL, CLOCK, id, [] );
        end
        
        o = get( o, varargin );
    end
    
end

