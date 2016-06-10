function jacobian = TakeJacobian( obj, x, u, w, dt, option )
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Dynamics class

% consider af/ax(x(t+delta_t/2))delta_t
switch(option)
    case ('F') % or state
        jacobian = [0 0 u(1)*cos(obj.x(3))*dt;
                    0 0 u(1)*sin(obj.x(3))*dt;
                    0 0                     0];
    case ('Gamma') % Gw is better, name w
        jacobian = [   dt      0       0;
                        0     dt       0;
                        0      0      dt];
    case ('Gu') % use u instead of Gu
        jacobian = [ sin(obj.x(3))*dt      0;
                    -cos(obj.x(3))*dt      0;
                                    0     dt];
end


end

