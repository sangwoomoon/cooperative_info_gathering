classdef Communication < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        Z % received package
        
    end % Properties
    
    methods
        function obj = Communication()
             obj = Declare(obj);
        end
        
        % Send Z to Network class
        SentPackage = SendPackage(obj, Z_agent, NETWORK);
        
        % Receive package from Network class
        ReceivePackage(obj, ReceivedPackge )
        
    end
    
end



