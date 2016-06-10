function x_pred = PredictState(obj, x, u_vec, w_vec, CLOCK)
%PREDICTSTATE generates the predicted state with respect to input array

x_curr = x;

% iterate with respect to finite horizon steps
for iStep = 1 : length(u_vec(1,:))
    x_next = obj.StateUpdate(x_curr, u_vec(:,iStep), w_vec(:,iStep), CLOCK);
    x_pred(:,iStep) = x_next;
    x_curr = x_next;
end

end
