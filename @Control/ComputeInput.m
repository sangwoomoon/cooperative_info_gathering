function o = ComputeInput( o, AGENT, option )
%COMPUTEINPUT Summary of this function goes here
%   Detailed explanation goes here

switch (option)
    case 'Lloyd'
        % u := kp * (C - X_agent)
        o.u = o.kp*(o.c-[AGENT.s(1),AGENT.s(3)]');        
    case 'Track'
        
end

o.hist.u(:,end+1) = o.u; % store input variable


end

