function obj = SetMatrices( obj, Y_k, F_k, Gamma_k, H_k, Q_k, R_k )
%SETMATRICES allocates matrices used for KF process

% set measurement matrix
obj.y = Y_k;

% set state transition matrix
obj.F = F_k;

% set process noise matrix
obj.Gamma = Gamma_k;

% set measurement matrix
obj.H = H_k;

% set process noise cov. matrix
obj.Q = Q_k;

% set measurement noise cov. matrix
obj.R = R_k;


end

