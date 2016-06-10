function jacobian = TakeJacobian( obj, u, dt, option )
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Dynamics class

switch(option)
    case ('F')
        jacobian = [1  0  0  0  0  0;
                    0  1  0  0  0  0;
                    0  0  1 dt  0  0;
                    0  0  0  1  0  0;
                    0  0  0  0  1  dt;
                    0  0  0  0  0  1];
    case ('Gamma')
        jacobian = [0.5*dt^2      0        0        0;
                        0     0.5*dt^2     0        0;   
                        0         0    0.5*dt^2     0;
                        0         0        dt       0;
                        0         0        0    0.5*dt^2
                        0         0        0        dt  ];
    case ('Gu')
        jacobian = [    0         0;
                        0         0;
                    0.5*dt^2      0;
                        dt        0;
                        0    0.5*dt^2
                        0         dt  ];
end


end

