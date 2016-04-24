classdef KalmanFilter < handle
    properties ( SetAccess = public, GetAccess = public )
    
        nState      % # of total states (for all agents - all targets if centralized KF)
        nY          % # of total measurements (for all agents' measurement if centralized KF)
        
        Xhat        % initial setting as a true state
        Phat        % can play with this; will not be issue with Info filter...
        Y           % current measurement set
        
        F           % [Ft1 0 0...0; 0 Ft2 ... 0; 0 ... 0 Fp1 ... 0; 0 ... 0 Fp2 ... 0] type
        Gamma       % [Gt1 0 0...0; 0 Gt2 ... 0; 0 ... 0 Gamp1 ... 0; 0 ... 0 Gamp2 ... 0] type
        Gu          % [zeros(2,4); Gu1, zeros(4,2); zeros(4,2),Gu2] type
        
        H           % range-bearing sensor jacobian           
        
        Q           % [Qt1 0 0...0; 0 Qt2 ... 0; 0 ... 0 Qp1 ... 0; 0 ... 0 Qp2 ... 0] type
        R           % [Rt1 0 0...0; 0 Rt2 ... 0; 0 ... 0 Rp1 ... 0; 0 ... 0 Rp2 ... 0] type
        V           % [vt1 0 0...0; 0 vt2 ... 0; 0 ... 0 ] type
        u           % [upvec1;upvec2] type
        
        bTrack      % tracking binary set : for task allocation         
        
        hist        % history of valuable
        plot        % plot handle

    end % Properties
    
    methods
        function o = KalmanFilter ( SIMULATION,AGENT,TARGET,CLOCK,option )
            o = Default( o, SIMULATION,AGENT,TARGET,CLOCK,option );
        end
        
        o = get( o, varargin );
    end
    
end



