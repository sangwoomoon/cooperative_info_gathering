function obj = Declare( obj )
%Default function is for initialization of linear dynamic model with sensor
%bias, so it is only usable for agents
%   It entails biases of sensors and position/velocity
%   x = [b_e, b_n, e, e_dot, n, n_dot]

    obj.spec = 'LinearBias';

end

