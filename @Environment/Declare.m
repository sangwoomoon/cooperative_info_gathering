function obj = Declare (obj, CLOCK)
    obj.LANDMARK = Target('LANDMARK','Linear');
    
    obj.LANDMARK.DYNAMICS.InitializeState([5.0,0,-2.5,0]');
    obj.LANDMARK.DYNAMICS.SetParameters(diag([0; 0]), 1e-4, 1e-6); % parameter setting order : bKFx, Q, RelTol, AbsTol (last two is for ODE45)

end