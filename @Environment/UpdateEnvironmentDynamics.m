function obj = UpdateEnvironmentDynamics(obj, CLOCK)

% propagate Landmark (set as stationary, with zero process noise)
if ~isempty(obj.LANDMARK)
    obj.LANDMARK.DYNAMICS.PropagateState(zeros(2,1),CLOCK); 
end

end