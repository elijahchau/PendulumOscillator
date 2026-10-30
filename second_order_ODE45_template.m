function second_order_ODE45_template()
    %define parameter values
    wn    =     %natural frequency value here
    zeta  =     %damping ratio value here

    %define initial conditions
    x0    =     %initial position here
    dxdt0 =     %initial velocity here
    Z0 = [x0;dxdt0]; %pack initial conditions into vector

    %define integration time range
    tmax =      %final integration time here
    t_range = [0,tmax]; %define time range

    %define rate function that encodes the ODE
    A = [0,1 ; -wn^2, -2*zeta*wn];
    rate_func = @(tin,Xin) A*Xin;
    
    %run ODE45
    [tlist,Zlist] = ode45(rate_func,t_range,Z0);
    
    %unpack result
    xvals = Zlist(:,1); dxdtvals = Zlist(:,2);
end