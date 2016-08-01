classdef FactorDDF < Fusion
    properties % ( SetAccess = public, GetAccess = public )
        
        xhat % xhat processed by DDF
        Phat % Phat processed by DDF
        
        xhatMgn % Marginalized KF state (extract target state from Xhat)
        PhatMgn % Marginalized Covariance Matrix of KF process
        
        xhatTemp % temporary stored Xhat DDF (at each step for sequential process)
        PhatTemp % temporary stored Phat DDF (at each step for sequential process)
        
        omega   % factorizing parameter
        
        M       % Information matrix
        m       % Information vector        
        
    end % Properties
    
    methods
        function obj = FactorDDF()
             obj@Fusion();
        end
        
        % initialize all properties into FDDF class
        Initialize(obj, xhat, Phat, nTargetState, agentID, nAgent);
    end
    
end



