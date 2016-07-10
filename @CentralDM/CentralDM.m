classdef CentralDM < handle
    properties ( SetAccess = public, GetAccess = public )
        
        s           % agent array
        
        act         % action profile
        actIdx      % action index
        
        util        % utility
        
        utilCnd
        
        alpha
        beta
        
        eta
        gamma
        
        rMove
        nMove
        
        hist        % history of valuable
        plot        % plot handle

    end % Properties
    
    methods
        function o = CentralDM ( SIMULATION,AGENT )
            o = Default( o, SIMULATION,AGENT );
        end
        
        o = get( o, varargin );
    end
    
end



