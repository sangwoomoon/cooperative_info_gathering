function djacobian = TakeJacobian( obj, jacobian, x, u, w, option )
%COMPUTEJACOBIAN computes the jacobian matrix
%   dependent on the sub-classes of Dynamics class


%       OBSOLETE FUNCTION


% consider af/ax(x(t+delta_t/2))delta_t
switch(option.jacobian)
    case ('state') 
        
        jacobian = reshape(jacobian,size(x),size(x));
        Amatrix = [ 0 0 -u(1)*cos(x(3));
                    0 0  u(2)*sin(x(3));
                    0 0         0     ];
                
        djacobian = Amatrix*jacobian;
        djacobian = reshape(djacobian,size(x)*size(x),1);
        
    case ('noise') 
        
        jacobian = reshape(jacobian,size(x),size(x));
        Amatrix = [ 0 0 -u(1)*cos(x(3));
                    0 0  u(2)*sin(x(3));
                    0 0         0     ];
                
        Dmatrix = eye(size(Amatrix));
        
        djacobian = Amatrix*jacobian + Dmatrix;
        djacobian = reshape(djacobian,size(x)*size(x),1);
        
    case ('input')
        
 
end


end

