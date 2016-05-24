function o = TakeJacobian( o, equation, value, option )
%TAKEJACOBIAN Summary of this function goes here
%   Detailed explanation goes here
 output = jacobian(equation,value);

 switch (option)
     case ('F')
         o.F = output;
     case ('Gamma')
         o.Gamma = output;
     case ('Gu')
         o.Gu = output;
 end

end

