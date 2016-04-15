classdef Lloyd < handle
    %LLOYD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties  ( SetAccess = public, GetAccess = public )
        
        C             %  centroid of voronoi cell (array, n by 2)
        
        hist
        plot
        
    end % Properties
    
    methods
        function o = Lloyd( CLOCK )
             o = Default(o, CLOCK );
        end
    end
    
end

