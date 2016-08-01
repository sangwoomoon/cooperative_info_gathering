function jacobian = TakeBiasJacobian( obj, option, Y_k )
%TAKETARGETJACOBIAN generates jacobian matrix for "SINGLE" target
%   this function is used for both centralized and local estimation

switch (option)
    case ('BiasState') % F_bias
        jacobian = [1  0;
                    0  1];
    case ('BiasNoise')
        jacobian = [1  0; % Gamma_bias
                    0  1];
    case ('BiasInput')
        jacobian = [0  0; % N/A
                    0  0];
    case ('BiasMeasure') % H_bias
        
        jacobian = [];
        jacobian_element = [1  0;  % bias measurement matrix for single target
                            0  1];
        
        for iMeasure = 1 : length(Y_k)
            jacobian = [jacobian; jacobian_element];
        end
        
end

end

