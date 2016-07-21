classdef Communication < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        range % possible communication range (N/A, but available in the future)
        Z % received package from Network class
        
    end % Properties
    
    methods
        function obj = Communication()
             obj = Declare(obj);
        end
        
        % Send Z to Network class
        SentPackage = SendPackage(obj, preferredAgent);
        
        % Receive package from Network class
        ReceivePackage(obj, ReceivedPackge )
        
    end
    
end



