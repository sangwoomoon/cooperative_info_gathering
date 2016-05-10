function o = PredictMeasurement(o,AGENT,TARGET)

% predict with zero noise

% range-bearing measurement 
o.y =[sqrt((TARGET.x(1)-AGENT.s(1))^2+(TARGET.x(3)-AGENT.s(3))^2);
        atan2(TARGET.x(3)-AGENT.s(3),TARGET.x(1)-AGENT.s(1))];

end