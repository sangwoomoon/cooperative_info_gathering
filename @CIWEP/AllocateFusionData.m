function obj = AllocateFusionData( obj, ESTIMATOR )
%ALLOCATEFUSIONDATA puts the fusion results to estimator

ESTIMATOR.xhat = obj.xhat;
ESTIMATOR.Phat = obj.Phat;

end

