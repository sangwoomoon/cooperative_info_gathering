function obj = MakeNoise( obj, option )
%MAKENOISE generates input noises with respect to classified dynamics
%   Will be useful to apply to various types of noises by receiving option
%   input parameter

switch (option)
    case ('zero')
        obj.w = zeros(4,1); % zero noise (used for prediction/estimation)
    otherwise
        obj.w = (mvnrnd(zeros(1,4),obj.Q,1))'; % noise for inputs
end

end

