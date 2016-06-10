function x_next = StateUpdate( obj, x_k, u_k, w_k, CLOCK )
%STATEUPDATE generates state with respect to delta_t in CLOCK. this
%function also uses the noise in Dynamics class as well as the current
%state as the initial state

% time update using ODE45 function
TimeProfile = 0:CLOCK.dt:CLOCK.dt;
[~, xout] = ode45(@(t,x)obj.StateDerivative(t,x,u_k,w_k), TimeProfile, x_k, obj.ODEoption);

x_next = xout(end,:)'; % should be transposed since xout is time profile

end

