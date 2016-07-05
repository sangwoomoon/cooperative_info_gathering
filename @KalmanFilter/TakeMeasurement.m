function G = TakeMeasurement( o, Xbar, s )
%TAKEMEASUREMENT Summary of this function goes here
%   Detailed explanation goes here

G = [sqrt((Xbar(1)-s(1))^2+(Xbar(3)-s(2))^2);
    atan2(Xbar(3)-s(2),Xbar(1)-s(1))];

end

