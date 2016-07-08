function o = Plot ( o )

figure(1)
set(o.plot.h.curr,'XData',o.s(1),'YData',o.s(2)); % plot current position

xD = [get(o.plot.h.path,'XData'),o.s(1)];
yD = [get(o.plot.h.path,'YData'),o.s(2)];

set(o.plot.h.path,'XData',xD,'YData',yD); % plot path

set(o.plot.h.num,'Position',[o.s(1)+0.3,o.s(2)-0.3]); % plot agent index

end