function o = AllocateCentroid( o, AGENT )
%ALLOCATECENTROID Summary of this function goes here
%   Detailed explanation goes here

for iAgent = 1 : length(AGENT)
   AGENT(iAgent).CONTROL.c = o.C(:,iAgent);
   AGENT(iAgent).CONTROL.hist.c(:,end+1) = AGENT(iAgent).CONTROL.c;
end

end

