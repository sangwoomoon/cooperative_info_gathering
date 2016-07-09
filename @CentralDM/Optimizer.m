function o = Optimizer( o, AGENT, ENVIRONMENT, option )
%OPTIMIZER Summary of this function goes here
%   Detailed explanation goes here

    x0 = nan(1,2*length(o.s(1,:)));
    lb = nan(1,2*length(o.s(1,:)));
    ub = nan(1,2*length(o.s(1,:)));

    for iAgent = 1 : length(o.s(1,:))
        x0(2*(iAgent-1)+1:2*iAgent) = length(o.s(:,iAgent));
    end
    
    options = []; % optimoptions('fmincon','MaxFunEvals',100000);
    
    for iAgent = 1 : length(o.s(1,:))
        lb(2*(iAgent-1)+1) = ENVIRONMENT.xlength(1);
        lb(2*iAgent) = ENVIRONMENT.ylength(1);
        
        ub(2*(iAgent-1)+1) = ENVIRONMENT.xlength(2);
        ub(2*iAgent) = ENVIRONMENT.ylength(2);
    end

    x = fmincon(@(x)o.ComputeGlobalUtility(x, AGENT, ENVIRONMENT, option), x0, [], [], [], [], lb, ub, [], options);
    
    for iAgent = 1 : length(o.s(1,:))
        o.s(:,iAgent) = x(2*(iAgent-1)+1:2*iAgent)';
    end

end

