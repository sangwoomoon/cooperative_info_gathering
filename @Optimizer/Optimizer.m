classdef Optimizer < handle
    %OPTIMIZER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties ( SetAccess = public, GetAccess = public )
        
        s
        
        Fp
        Gu
        
        u % input parameter (for optimizer)
        
        vp
        
        x
        
        Ft
        
        
        cost
        
        hist
        plot
        
    end
    
    methods
        function o = Optimizer(  )
            o = Default(o );
        end
        
        o = get( o, varargin );
    end
    
end

