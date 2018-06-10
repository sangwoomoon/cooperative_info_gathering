function stateNext = LinModel(stateNow,param)

    nState = length(stateNow);
    
    % make process noise: Q is zero when handling truth
    procNoise = mvnrnd(zeros(nState,1),param.Q)';
    
    % linear dynamics model
    stateNext = param.F*stateNow + procNoise;
    
end