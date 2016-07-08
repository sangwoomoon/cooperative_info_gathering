function o = Plot(o)


figure(1)

for iAgent = 1 : length(o.s(1,:))
    set(o.plot(iAgent).h.curr,'XData',o.s(1,iAgent),'YData',o.s(2,iAgent)); % plot current position
    
    xD = [get(o.plot(iAgent).h.path,'XData'),o.s(1,iAgent)];
    yD = [get(o.plot(iAgent).h.path,'YData'),o.s(2,iAgent)];
    
    set(o.plot(iAgent).h.path,'XData',xD,'YData',yD); % plot path
    
    set(o.plot(iAgent).h.num,'Position',[o.s(1,iAgent)+0.3,o.s(2,iAgent)-0.3]); % plot agent index
end

end