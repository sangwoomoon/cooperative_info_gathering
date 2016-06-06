function obj = MakeNoise( obj )
%MAKENOISE Summary of this function goes here
%   Detailed explanation goes here

obj.v = (mvnrnd(zeros(1,4),obj.Q,1))'; % noise for inputs

end

