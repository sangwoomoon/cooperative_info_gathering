function o = Default( o, SIMULATION, CLOCK )
%DE Summary of this function goes here
%   Detailed explanation goes here

    o.s_sym = sym('s_sym',[6,1]);
    o.v_sym = sym('v_sym',[4,1]);

    syms f_sym;
    
    % To be defined by users
    f_sym = symfun([o.s_sym(1)+CLOCK.dt_sym*o.v_sym(1);...
        o.s_sym(2)+CLOCK.dt_sym*o.v_sym(2);...
        o.s_sym(3)+o.s_sym(4)*CLOCK.dt_sym+0.5*CLOCK.dt_sym^2+0.5*CLOCK.dt_sym^2*CONTROL.u_sym(1);...
        ], o.s_sym);
    
    o.TakeJacobian(f_sym,o.s_sym,'F');
    o.TakeJacobian(f_sym,CONTROL.u_sym,'Gu');
    o.TakeJacobian(f_sym,o.v_sym,'Gamma');
    
    o.Q = diag([0.01 0.01 0.01]);

o.F = blkdiag(eye(2),[1 CLOCK.dt; 0 1],[1 CLOCK.dt; 0 1]);

o.Gamma = [    CLOCK.dt               0   ;
                    0          CLOCK.dt  ;
          0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; % make the separate Q matrix

o.Gu = [            0                0   ;
                    0                0   ;
          0.5*CLOCK.dt^2             0   ;
              CLOCK.dt               0   ;
                     0    0.5*CLOCK.dt^2 ;
                     0        CLOCK.dt  ]; % make the separate Q matrix


o.Q = diag([0.01 0.01]);

end

