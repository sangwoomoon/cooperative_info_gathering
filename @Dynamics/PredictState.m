function x_out = PredictState(obj, CONTROL, CLOCK, N)

for iStep = 1 : N
    x_out = obj.StateDerivate(obj, CLOCK, CONTROL);
end

end
