function obj = SetParameters( obj, binaryStateInput, QInput, RelTol, AbsTol )
%SETPARAMETERS Summary of this function goes here
%   Detailed explanation goes here

    % set process noise parameters (Q)
    obj.Q = QInput;
    
    % set binary array to determine elements in state for Kalman Filtering
    % process
    obj.bKFx = binaryStateInput;
    
    % set ODE45 error options
    obj.ODEoption = odeset('RelTol',RelTol,'AbsTol',AbsTol);

end

