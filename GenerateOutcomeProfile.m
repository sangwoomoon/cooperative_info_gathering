function [event,eventNum,outcomeNum,outcome] = GenerateOutcomeProfile(event,horizTimeStep)

eventNum = length(event);
outcomeNum = eventNum^horizTimeStep;

for iAct = 1 : horizTimeStep
    assignDen = outcomeNum/eventNum^iAct;
    repeatNum = outcomeNum/assignDen;
    for jAct = 1 : repeatNum
        eventElem = event(rem(jAct,eventNum)+1);
        for kAct = 1 : assignDen
            outcome(iAct,assignDen*(jAct-1)+kAct) = eventElem;
        end
    end
end

end