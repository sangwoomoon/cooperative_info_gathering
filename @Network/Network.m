classdef Network < handle
    %NETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        graph % binary matix (sender - receiver, could be asymmetric/symmetric)
        
        prob % 100% - 0% of communication (could be usable for noize/binary)
        range % criteria of communication between agents (distance-based)
        
        Z % received data package from agents
    end
    
    methods
        function o = Network( range )
            o = Default( o, range );
        end
        
        o = get( o, varargin );
    end
    
end

