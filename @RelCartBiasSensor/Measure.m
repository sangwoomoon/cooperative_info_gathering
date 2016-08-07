function obj = Measure( obj, p_k, TARGET, LANDMARK, current_time )
%MEASURE actually measures agents/targets (represented as x in inputs) at
%time t which is also given among inputs
%   take measure action from the converted state (as linear-bias) to given
%   specified-

%%Simulate relative position measurements to target from platform
obj.v = mvnrnd(zeros(1,2),obj.R,1);

idxTarget = 0;
for iTarget = 1 : length(TARGET)
    if obj.bTrack(iTarget) == 1
        idxTarget = idxTarget + 1;
        
        % BEWARE OF COORDINATE (GLOBAL)
        delx = TARGET(iTarget).DYNAMICS.GetPosition();
    
        obj.meas(idxTarget).id = TARGET(iTarget).id;
        obj.meas(idxTarget).y(1,1) = obj.bias(1) + delx(1) + obj.v(1); %relative easting + bias
        obj.meas(idxTarget).y(2,1) = obj.bias(2) + delx(2) + obj.v(2); %relative northing + bias
    end
    
end

for iLandmark = 1 : length(LANDMARK)
    obj.meas(sum(obj.bTrack)+iLandmark).id = LANDMARK.id;
    obj.meas(sum(obj.bTrack)+iLandmark).y(1,1) = obj.bias(1) + obj.v(1); % bias + noise
    obj.meas(sum(obj.bTrack)+iLandmark).y(2,1) = obj.bias(2) + obj.v(2); % bias + noise
end

%%Store measurement to history
obj.hist.meas{end+1} = obj.meas;

% stamp current time
obj.hist.stamp(length(obj.hist.meas)) = current_time;


end

