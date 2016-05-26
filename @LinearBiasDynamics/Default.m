function o = Default( o, CONTROL, CLOCK, id )
%Default function is for initialization of linear dynamic model with sensor
%bias, so it is only usable for agents
%   It entails biases of sensors and position/velocity
%   s = [b_e, b_n, e, e_dot, n, n_dot]

    o.spec = 'LinearBias';

    o.s_sym = sym('s_sym',[6,1]);
    o.v_sym = sym('v_sym',[4,1]);
    
    % To be defined by users
    o.Eqn = symfun([o.s_sym(1)+CLOCK.dt_sym*o.v_sym(1);...
                    o.s_sym(2)+CLOCK.dt_sym*o.v_sym(2);...
                    o.s_sym(3)+o.s_sym(4)*CLOCK.dt_sym+0.5*CLOCK.dt_sym^2*o.v_sym(3)+0.5*CLOCK.dt_sym^2*CONTROL.u_sym(1);...
                    o.s_sym(4)+CLOCK.dt_sym*o.v_sym(3)+CLOCK.dt_sym*CONTROL.u_sym(1);...
                    o.s_sym(5)+o.s_sym(6)*CLOCK.dt_sym+0.5*CLOCK.dt_sym^2*o.v_sym(4)+0.5*CLOCK.dt_sym^2*CONTROL.u_sym(2);...
                    o.s_sym(6)+CLOCK.dt_sym*o.v_sym(4)+CLOCK.dt_sym*CONTROL.u_sym(2)], o.s_sym);
    
    o.TakeJacobian(o.Eqn,o.s_sym,'F');
    o.TakeJacobian(o.Eqn,CONTROL.u_sym,'Gu');
    o.TakeJacobian(o.Eqn,o.v_sym,'Gamma');
    
    o.Q = diag([0.01 0.01 0.01 0.01]);
    
    o.plot.statecolor = rand(1,3);
    o.plot.marker = ['o';'x']; % start; end
    o.plot.markersize = 7;
    o.plot.line = '--';
    o.plot.linewidth = 3;
    
    o.plot.legend = [{strcat('Agent ',num2str(id))},...
        {strcat('Agent ',num2str(id),' start')},...
        {strcat('Agent ',num2str(id),' end')}];

end

