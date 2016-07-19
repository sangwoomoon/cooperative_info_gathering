function obj = SetParameters( obj, QInput, RelTol, AbsTol )
%SETPARAMETERS Summary of this function goes here
%   Detailed explanation goes here

    % set process noise parameters (Q)
    obj.Q = QInput;
    
    % set ODE45 error options
    obj.ODEoption = odeset('RelTol',RelTol,'AbsTol',AbsTol);

end

