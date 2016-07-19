classdef RelCartCoordSensor < Sensor
    %LINEARMEASURE is a sub-class of Meausrement class
    %   measurement : [abs_e, abs_n, rel_e, rel_n]'
    
    properties
        
    end
    
    methods
        
        function obj = RelCartCoordSensor()
            obj@Sensor();
        end
        
        % Take measurement for other agents/targets
        Measure(obj, s, TARGET, LANDMARK, current_time);
        
        % make measurement noise
        MakeNoise(obj, option);
        
        % Initialize measurement data and its history
        InitializeData(obj);
        
        % Plot History of states 
        Plot(obj);
    end
    
end

