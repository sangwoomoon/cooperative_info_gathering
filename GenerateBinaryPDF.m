%-------------------------------------------------------
% Binary Distribution generation with respect to sensor model
%
% IT IS GENERATED IN A CENTRALIZED MANNER
%-------------------------------------------------------
function pdf = GenerateBinaryPDF(pt,paramSensor,paramAgent)

nAgent = length(paramAgent);

% check whether particle is in the sensing region of any of agents
bPtInside = 0;
for iAgent = 1:nAgent
    if IsInCircleBasedRegion(pt,paramAgent(iAgent).s,paramSensor.regionRadius)
        bPtInside = 1;
    end
end

% when the particle is inside of sensing retion, it follows the beta
% otherwise, the probability should be zero based on the detection
% event
if bPtInside
    pdf = paramSensor.detectBeta;
else
    pdf = 0;
end

end