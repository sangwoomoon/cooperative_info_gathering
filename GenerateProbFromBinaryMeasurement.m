function prob = GenerateProbFromBinaryMeasurement(y,sensor,agentPos,targetPos)

if y == 1 % when sensor does detect
    if targetPos >= agentPos-sensor.regionRadius && targetPos <= agentPos+sensor.regionRadius % when the particle is within detected region
        prob = sensor.DetectBeta;
    else
        prob = 1-sensor.DetectBeta;
    end
else
    if targetPos >= agentPos-sensor.regionRadius && targetPos <= agentPos+sensor.regionRadius % when the particle is within detected region
        prob = 0;
    else
        prob = 1;
    end
end

end