classdef Fusion < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        spec    % fusion specification
        
        bSend2others % The determinent array for flags to send it to other agents
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Fusion()
             obj = Declare(obj);
        end
        
        
    end
    
end



