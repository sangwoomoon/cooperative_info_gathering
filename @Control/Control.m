classdef Control < handle
    properties % ( SetAccess = public, GetAccess = public )
                
        u       % acceleration input [eddot,nddot]
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Control()
             obj = Declare(obj);
        end
        
        obj = get( obj, varargin );
    end
    
end



