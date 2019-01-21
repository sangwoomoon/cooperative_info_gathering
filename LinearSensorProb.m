function prob = LinearSensorProb(measNow,ptNow,paramSensor)

nState = length(ptNow(:,1));
R = paramSensor.R;
% sensor directly observes target's state

switch nState
    
    case 2 % 2D position
        
        % 1. substract measNow from ptNow
        ptDiffX = ptNow(1,:) - measNow(1);
        ptDiffY = ptNow(2,:) - measNow(2);
        
        % 2. compute exponential part
        ptDomainExpSuper = -1/2.*(1/R(1,1).*ptDiffX.^2 + 1/R(2,2).*ptDiffY.^2);

    case 3 % 3D position
        
        % 1. substract measNow from ptNow
        ptDiffX = ptNow(1,:) - measNow(1);
        ptDiffY = ptNow(2,:) - measNow(2);
        ptDiffZ = ptNow(3,:) - measNow(3);
        
        % 2. compute exponential part
        ptDomainExpSuper = -1/2.*(1/R(1,1).*ptDiffX.^2 + 1/R(2,2).*ptDiffY.^2 + 1/R(3,3).*ptDiffZ.^2);        
        
end

% 3. compute remain parts. the results is pdf.
prob = (1/sqrt(2*pi*det(R))).*exp(ptDomainExpSuper);

end