function prob = RFProb(measNow,paramAgent,ptNow,paramSensor)

nState = length(ptNow(:,1));
R = paramSensor.R;
agentState = paramAgent.s;
paramSensorIdeal.R = zeros(size(R));

switch nState

    case 4 % 2D position with RF info
        
        % 1. substract measNow from h(ptNow,s)
        ptDiff = RFMeasurement(ptNow,agentState,paramSensorIdeal) - measNow;
        
        % 2. compute exponential part
        ptDomainExpSuper = -1/2.*(1/R.*ptDiff.^2);

    case 5 % 3D position     
        
end

% 3. compute remain parts. the results is pdf.
prob = (1/sqrt(2*pi*det(R))).*exp(ptDomainExpSuper);


end