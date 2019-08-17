function H = ComputeBearMeasMatrix(s,xhat)

nState = length(xhat);

switch nState
    case 2 % 2D static target
        r = norm(s(1:2) - xhat);
        H = [-(xhat(1)-s(1))/r^2 (xhat(2)-s(2))/r^2];
    case 3 % 3D static
        
    case 4 % 2D moving
        
    case 6 % 3D moving

end
