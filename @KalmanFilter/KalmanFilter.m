classdef KalmanFilter < Estimator
    properties ( SetAccess = public, GetAccess = public )
    
        y           % current measurement
        
        F           % state transition matrix (given by AGENT class, added AGENT.SENSOR class if bias is considered for estimation)
        
        Gamma       % noise matrix (given by AGENT class, added AGENT.SENSOR class if bias is considered for estimation)
        Q           % process noise covariacne matrix (given by AGENT class, added AGENT.SENSOR class if bias is considered for estimation)
        
        H           % measurement matrix (given by AGENT.SENSOR class)
        R           % [Rt1 0 0...0; 0 Rt2 ... 0; 0 ... 0 Rp1 ... 0; 0 ... 0 Rp2 ... 0] type
        
        Gu          % input matrix (given by AGENT class)
        u           % [upvec1;upvec2] type

    end % Properties
    
    methods
        function obj = KalmanFilter()
            obj@Estimator();
        end
        
    end
    
end



