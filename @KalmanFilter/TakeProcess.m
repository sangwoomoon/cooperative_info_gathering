function obj = TakeProcess( obj, SENSOR, CLOCK )
%TAKEPROCESS takes estimation process given by the spec of estimation

% Take matrix for estimation which is belonged and excuted
% in the Kalman Filter sub-class
obj.F = obj.GatherJacobian(SENSOR,CLOCK,'state');
obj.Gamma = obj.GatherJacobian(SENSOR,CLOCK,'noise');

obj.Y = SENSOR.GatherMeasurements();
obj.H = SENSOR.TakeJacobian();

obj.Q = obj.GatherProcNoiseCovMatrix(SENSOR);
obj.R = SENSOR.GatherMeasNoiseCovMatrix();


%%Time update
xbar = obj.F*obj.xhat;
Pbar = obj.F*obj.Phat*obj.F' + obj.Gamma*obj.Q*obj.Gamma';

%%Measurement update
innov = obj.Y - obj.H*xbar;
S = obj.H*Pbar*obj.H' + obj.R;
W = Pbar*obj.H' / S; % Kalman Gain (den : inversed form of S matrix)

xhat = xbar + W*innov;
Phat = (eye(obj.nState) - W*obj.H)*Pbar*(eye(obj.nState) - W*obj.H)' + W*obj.R*W'; % Joseph form to prevent from numerical issues

%%Store and update
obj.xhat = xhat;
obj.Phat = Phat;

obj.hist.xhat(:,end+1) = obj.xhat;
obj.hist.Phat(:,:,end+1) = obj.Phat;

end

