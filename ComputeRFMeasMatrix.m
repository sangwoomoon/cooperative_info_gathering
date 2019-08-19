function H = ComputeRFMeasMatrix(s,xhat)

nState = length(xhat);

switch nState
    case 4 % 2D static target with RF states
        
        % MEASUREMENT MATRIX (JACOBIAN) info ---------------------------
        % H = [ -10*alpha*(x-s_x)/(log(10)*r^2), -10*alpha*(y-s_y)/(log(10)*r^2), 10/(k_0*log(10)), -5*log(r^2)/log(10)]
        % --------------------------------------------------------------
        
        r = norm(s(1:2) - xhat);
        
        % d/dx
        h1 = -10*xhat(4)*(xhat(1)-s(1))/(log(10)*r^2);
        % d/dy
        h2 = -10*xhat(4)*(xhat(2)-s(2))/(log(10)*r^2);
        % d/dk
        h3 = 10/(xhat(3)*log(10));
        % d/dalpha
        h4 = -5*log(r^2)/log(10);
        
        H = [h1 h2 h3 h4];
        
    case 5 % 3D static
        
    case 6 % 2D moving
        
    case 8 % 3D moving

end
