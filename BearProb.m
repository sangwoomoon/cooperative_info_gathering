function prob = BearProb(measNow,paramAgent,ptNow,paramSensor)

nState = length(ptNow(:,1));
R = paramSensor.R;
agentState = paramAgent.s;
paramSensorIdeal.R = zeros(size(R));

switch nState
    
    case 2 % 2D position
        
        % 1. substract measNow from h(ptNow,s)
        ptDiff = BearMeasurement(ptNow,agentState,paramSensorIdeal) - measNow;
        
        % 2. compute exponential part
        ptDomainExpSuper = -1/2.*(1/R.*ptDiff.^2);

    case 3 % 3D position     
        
end

% 3. compute remain parts. the results is pdf.
prob = (1/sqrt(2*pi*det(R))).*exp(ptDomainExpSuper);


end