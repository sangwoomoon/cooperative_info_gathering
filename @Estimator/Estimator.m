classdef Estimator < handle
    properties ( SetAccess = public, GetAccess = public )
    
        spec        % characteristics of estimation process
        
        TARGET      % estimated targets (array form)
        bias        % estimated bias
        
        nState      % # of states 
        nY          % # of measurements 
        
        xhat        % estimated state
        Phat        % information
        
        hist        % history of valuable
        plot        % plot handle

    end % Properties
    
    methods
        function obj = Estimator()
            obj = Declare(obj);
        end
        
    end
    
end



