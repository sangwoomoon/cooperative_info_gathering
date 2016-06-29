classdef Voronoi < TaskAllocation
    %VORONOI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties  ( SetAccess = public, GetAccess = public )
        
        vertices      %  vertices of Voronoi partition
        cell          %  voronoi cell (structure)
        
    end % Properties
    
    methods
        function o = Voronoi( TARGET, CLOCK )
             o@TaskAllocation( TARGET, CLOCK );
        end
    end
    
end

