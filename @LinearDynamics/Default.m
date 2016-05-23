function o = Default( o, SIMULATION, CLOCK )
%DE Summary of this function goes here
%   Detailed explanation goes here
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

