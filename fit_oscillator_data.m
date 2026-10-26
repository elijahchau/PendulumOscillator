function signal_params = fit_oscillator_data(t,x)
    f_cost = @(q) cost_func(q,t,x);
    f_grad = @(q) compute_numerical_jacobian(f_cost,q)';

    inv_time_constants = grad_descent_with_line_search(f_grad, [estimate_freq_from_fft(t,x);0] ,1e-6);
    coeff_out = regress_linear_parameters(inv_time_constants,t,x);

    C = coeff_out(1);
    S = coeff_out(2);
    D = coeff_out(3);
    omega_d = inv_time_constants(1);
    sigma = inv_time_constants(2);
    omega_n = sqrt(omega_d^2+sigma^2);
    zeta = sigma/omega_n;

    y_func = @(t_in) D+(C*cos(omega_d*t_in)+S*sin(omega_d*t_in)).*exp(-sigma*t_in);
    dydt_func = @(t_in) omega_d*(-C*sin(omega_d*t_in)+S*cos(omega_d*t_in)).*exp(-sigma*t_in) ...
                       -sigma*(C*cos(omega_d*t_in)+S*sin(omega_d*t_in)).*exp(-sigma*t_in);

    y0 = y_func(0);
    dydt0 = dydt_func(0);

    signal_params = struct();
    signal_params.C = C;
    signal_params.S = S;
    signal_params.D = D;
    signal_params.omega_d = omega_d;
    signal_params.sigma = sigma;
    signal_params.omega_n = omega_n;
    signal_params.zeta = zeta;
    signal_params.y0 = y0;
    signal_params.dydt0 = dydt0;

    signal_params.t = t;
    signal_params.x_approx = D+(C*cos(omega_d*t)+S*sin(omega_d*t)).*exp(-sigma*t);

    print_results(signal_params);
end

function print_results(signal_params)

    C = signal_params.C;
    S = signal_params.S;
    omega_d = signal_params.omega_d;
    sigma = signal_params.sigma;
    omega_n = signal_params.omega_n;
    zeta = signal_params.zeta;
    y0 = signal_params.y0;
    dydt0 = signal_params.dydt0;

    str_out1 = '';
    str_out1 = [str_out1,'y(t) = '];
    str_out1 = [str_out1,'('];
    str_out1 = [str_out1,num2str(C,3)];
    str_out1 = [str_out1,'*cos('];
    str_out1 = [str_out1,num2str(omega_d,3)];
    str_out1 = [str_out1,'*t)'];
    str_out1 = [str_out1,select_sign(S),num2str(abs(S),3)];
    str_out1 = [str_out1,'*sin('];
    str_out1 = [str_out1,num2str(omega_d,3)];
    str_out1 = [str_out1,'*t))'];
    str_out1 = [str_out1,'*e^('];
    str_out1 = [str_out1,num2str(-sigma,3)];
    str_out1 = [str_out1,'*t)'];

    str_out2 = '';
    str_out2 = [str_out2,'y_func = @(t_in) '];
    str_out2 = [str_out2,'('];
    str_out2 = [str_out2,num2str(C,3)];
    str_out2 = [str_out2,'*cos('];
    str_out2 = [str_out2,num2str(omega_d,3)];
    str_out2 = [str_out2,'*t_in)'];
    str_out2 = [str_out2,select_sign(S),num2str(abs(S),3)];
    str_out2 = [str_out2,'*sin('];
    str_out2 = [str_out2,num2str(omega_d,3)];
    str_out2 = [str_out2,'*t_in))'];
    str_out2 = [str_out2,'.*exp('];
    str_out2 = [str_out2,num2str(-sigma,3)];
    str_out2 = [str_out2,'*t_in);'];

    q1 = 2*zeta*omega_n;
    q2 = omega_n^2;

    str_out3 = '';
    str_out3 = [str_out3,'(d^2y/dt^2)'];
    str_out3 = [str_out3,num2str(q1,3)];
    str_out3 = [str_out3,'*(dy/dt)'];
    str_out3 = [str_out3,select_sign(q2),num2str(abs(q2),3)];
    str_out3 = [str_out3,'*(y) = 0'];

    str_out4 = '';
    str_out4 = [str_out4,'y (t=0) = ',num2str(y0,3)];
    str_out4 = [str_out4,' dy/dt (t=0) = ',num2str(dydt0,3)];

    str_out5 = '';
    str_out5 = [str_out5,'rate_func = @(t_in, Y_in) ['];
    str_out5 = [str_out5,'Y_in(2) ;'];
    str_out5 = [str_out5,num2str(-q2,3)];
    str_out5 = [str_out5,'*Y_in(1)'];
    str_out5 = [str_out5,' ',select_sign(-q1),' '];
    str_out5 = [str_out5,num2str(abs(q1),3)];
    str_out5 = [str_out5,'*Y_in(2)];'];

    str_out6 = '';
    str_out6 = [str_out6,'Y0 = [',num2str(y0,3),' ; ',num2str(dydt0,3),'];'];
    

    fprintf(1,'\n');
    disp('Analytical Expression:');
    disp(str_out1);
    fprintf(1,'\n');
    disp('MATLAB Function Definition');
    disp(str_out2);
    fprintf(1,'\n');
    disp('2nd Order ODE');
    disp(str_out3);
    fprintf(1,'\n');
    disp('Initial Conditions:');
    disp(str_out4);
    fprintf(1,'\n');
    disp('Rate Function for ODE45:');
    disp(str_out5);
    fprintf(1,'\n');
    disp('Initial Conditions for ODE45:');
    disp(str_out6);
    fprintf(1,'\n');
end


function str_out = select_sign(n)
    if n>=0
        str_out = '+';
    else
        str_out = '-';
    end
end

function omega_guess = estimate_freq_from_fft(t,x)
    fft_signal = abs(fft(x));
    fft_signal(1) = 0;

    [~,discrete_freq] = max(fft_signal);

    discrete_freq = discrete_freq-1;

    if discrete_freq>length(x)/2
        discrete_freq = length(x)-discrete_freq;
    end
    
    omega_guess = 2*pi*discrete_freq/(max(t)-min(t));
end


function q = grad_descent_with_line_search(f_grad,q,threshold)
    count = 0;
    G_norm = 1;

    while  count<1e3 && G_norm>threshold
        count = count+1;
        [q,G_norm] = line_search_subroutine(f_grad,q);
    end
end

function [q_next,gradient_norm] = line_search_subroutine(f_grad,q_in)
        current_gradient = f_grad(q_in);

        if norm(current_gradient)>.1*norm(q_in)
            current_gradient=(.1*norm(q_in)/norm(current_gradient))*current_gradient;
        end

        gradient_hat = current_gradient/norm(current_gradient);
        
        s_guess = 1;
        q_next = q_in-s_guess*current_gradient;
        gradient_next = f_grad(q_in-s_guess*current_gradient);

        dfds_start = dot(gradient_next,gradient_hat);
        
        if dfds_start>0
            s_min = s_guess;

            s_guess = s_guess*2;

            while dot(f_grad(q_in-s_guess*current_gradient),gradient_hat)>0
                s_guess = s_guess*2;
            end
            
            s_max = s_guess;
        end

        if dfds_start<0
            s_min = 0;
            s_max = s_guess;
        end

        if dfds_start==0
            s_min = s_guess;
            s_max = s_guess;
        end

        while abs(s_max-s_min)>1e-2
            s_guess = (s_max+s_min)/2;

            q_next = q_in-s_guess*current_gradient;
            gradient_next = f_grad(q_in-s_guess*current_gradient);
            dfds = dot(gradient_next,gradient_hat);

            if dfds>=0
                s_min = s_guess;
            end

            if dfds<=0
                s_max = s_guess;
            end


        end

        gradient_norm = norm(gradient_next);
end

function error_out = cost_func(param_vec,t_data,x_data)
    omega = param_vec(1);
    sigma = param_vec(2);

    if size(t_data,1)==1
        t_data = t_data';
    end
    
    if size(x_data,1)==1
        x_data = x_data';
    end

    XC = cos(omega*t_data).*exp(-sigma*t_data);
    XS = sin(omega*t_data).*exp(-sigma*t_data);
    XO = ones(length(t_data),1);
 
    M = [XC,XS,XO];
    
    b = (M'*M)\(M'*x_data);

    e_vals = x_data-M*b;

    error_out = .5*(e_vals'*e_vals)/length(t_data);
end

function coeff_out = regress_linear_parameters(param_vec,t_data,x_data)
    omega = param_vec(1);
    sigma = param_vec(2);

    if size(t_data,1)==1
        t_data = t_data';
    end
    
    if size(x_data,1)==1
        x_data = x_data';
    end

    XC = cos(omega*t_data).*exp(-sigma*t_data);
    XS = sin(omega*t_data).*exp(-sigma*t_data);
    XO = ones(length(t_data),1);
    
    M = [XC,XS,XO];
    
    coeff_out = (M'*M)\(M'*x_data);
end


function jacobian_out = compute_numerical_jacobian(fun,X)
    delta_x = 1e-6;

    Y = fun(X);

    jacobian_out = zeros(length(Y),length(X));

    for n = 1:length(X)
        X0 = X;
        X1 = X;

        X0(n)=X0(n)-delta_x/2;
        X1(n)=X1(n)+delta_x/2;

        Y0 = fun(X0);
        Y1 = fun(X1);

        jacobian_out(:,n) = (Y1-Y0)/delta_x;
    end
end