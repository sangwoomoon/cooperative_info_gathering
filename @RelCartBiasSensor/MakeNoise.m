function obj = MakeNoise( obj, option )
%MAKENOISE generates measurement noise with respect to classified
%measurement model
%   Will be useful to apply to various types of noises by receiving option
%   input parameter

switch (option)
    case ('Gaussian')
        obj.v = (mvnrnd(zeros(1,2),obj.R,1))'; % Gaussian measurement noise vector
end

end

