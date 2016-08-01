function jacobian = TakeTargetJacobian( obj, Y_k )
%TAKETARGETJACOBIAN generates jacobian matrix for "SINGLE" target
%   this function is used for both centralized and local estimation

jacobian = [];

for iMeasure = 1 : length(Y_k)
    
    jacobian_element = [1  0  0  0; % H for single target
        0  0  1  0];
    
    if length(Y_k(iMeasure).id) == 1 % if measured one is target
        jacobian = blkdiag(jacobian,jacobian_element); % since homogeneous measuring for targets
    else % if measured one is landmark
        jacobian = [jacobian;zeros(length(Y_k(iMeasure).y),length(jacobian(1,:)))]; % since states of landmark are not concerned
    end
end

end

