function o = Default( o, SIMULATION, CONTROL, CLOCK )
%Default Summary of this function goes here
%   Detailed explanation goes here
    
    o.s_sym = sym('s_sym',[3,1]);
    o.v_sym = sym('v_sym',[3,1]);
    
    % To be defined by users
    o.Eqn = symfun([o.s_sym(1)+CONTROL.u_sym(1)*cos(o.s_sym(3))*CLOCK.dt_sym + o.v_sym(1);...
                    o.s_sym(2)+CONTROL.u_sym(1)*sin(o.s_sym(3))*CLOCK.dt_sym + o.v_sym(2);...
                    o.s_sym(3)+CONTROL.u_sym(2)*CLOCK.dt_sym + o.v_sym(3)], o.s_sym);
    
    o.TakeJacobian(o.Eqn,o.s_sym,'F');
    o.TakeJacobian(o.Eqn,CONTROL.u_sym,'Gu');
    o.TakeJacobian(o.Eqn,o.v_sym,'Gamma');
    
    o.Q = diag([0.01 0.01 0.01]);
    
end

