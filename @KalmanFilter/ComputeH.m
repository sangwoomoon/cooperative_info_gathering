function o = ComputeH( o, s, x )
%COMPUTEH Summary of this function goes here
%   Detailed explanation goes here

H_11 = (x(1)-s(1))/sqrt((x(1)-s(1))^2+(x(3)-s(2))^2);
H_12 = (x(3)-s(2))/sqrt((x(1)-s(1))^2+(x(3)-s(2))^2);
H_21 = (s(2)-x(3))/((x(1)-s(1))^2+(x(3)-s(2))^2);
H_22 = (x(1)-s(1))/((x(1)-s(1))^2+(x(3)-s(2))^2);

dH = [H_11, 0, H_12, 0;
      H_21, 0, H_22, 0];
    
o.H = [o.H;dH];

end

