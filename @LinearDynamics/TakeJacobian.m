function jacobian = TakeJacobian( obj, u, dt, option )
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Dynamics class

switch(option)
    case ('state')
        jacobian = [1 dt  0  0;
                    0  1  0  0;
                    0  0  1  dt;
                    0  0  0  1];
    case ('noise')
        jacobian = [0.5*dt^2      0;
                        dt        0;
                        0    0.5*dt^2
                        0        dt  ];
                    
    case ('input')
        jacobian = [0.5*dt^2      0;
                        dt        0;
                        0    0.5*dt^2
                        0        dt  ];
end

end

