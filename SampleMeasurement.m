% sample measurement: for weight update of PDF approach and
% likelihood function of Ryan's approach
function ySample = SampleMeasurement(pdf,nState,dRefPt)

nPt = numel(pdf); % same as prod(size(pdf))

pdf = reshape(pdf,nPt,1);
cdf = cumsum(pdf);

yIdx = find(rand/(dRefPt^nState) <= cdf,1);
if isempty(yIdx)
    yProb = 0;
else
    yProb = pdf(yIdx);
end
ySample = binornd(1,yProb);

end