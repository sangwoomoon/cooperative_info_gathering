function o = TakeAction( o )
%TAKEACTION Summary of this function goes here
%   Detailed explanation goes here

[~,I] = max(o.utilCnd(:));

[I1,I2,I3,I4] = ind2sub(size(o.utilCnd),I);

U = o.utilCnd(I1,I2,I3,I4);

o.s = [squeeze(o.act(1,I1,:)), squeeze(o.act(2,I2,:)), squeeze(o.act(3,I3,:)), squeeze(o.act(4,I4,:))];
o.util = U;
o.actIdx = [I1;I2;I3;I4];

o.hist.s(:,:,end+1) = o.s;
o.hist.util(end+1) = o.util;
o.hist.actIdx(:,end+1) = o.actIdx;

end

