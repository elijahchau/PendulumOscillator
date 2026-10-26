%template code for how to use oscillator fitting function
function fit_oscillator_data_usage_example()

    %create some noisy test data
    %define natural frequency and damping ratio
    wn = 3; z = .1;
    %compute decay rate and damped frequency
    sigma = z*wn; wd = wn*sqrt(1-z^2);
    %define coefficients in sum
    C = 4; S = -5; D = 3.7;
    %define amount of noise to add
    q =.05*sqrt(C^2+S^2);

    %define a range of time values
    t_list = linspace(0,10,201);

    %compute sample data with noise
    X_list = D+(C*cos(wd*t_list)+S*sin(wd*t_list)).*exp(-sigma*t_list) + q*randn(size(t_list));

    %use underdamped fitting tool
    underdamped_response_params = fit_oscillator_data(t_list,X_list);

    %display fit parameters.
    disp(underdamped_response_params);

    %extract fit parameters
    C_fit = underdamped_response_params.C;
    S_fit = underdamped_response_params.S;
    D_fit = underdamped_response_params.D;
    sigma_fit = underdamped_response_params.sigma;
    wd_fit = underdamped_response_params.omega_d;

    %compute fit points
    X_fit = D_fit+(C_fit*cos(wd_fit*t_list)+S_fit*sin(wd_fit*t_list)).*exp(-sigma_fit*t_list);

    %plot results
    figure(1)
    hold on
    plot(t_list,X_list,'ro','markerfacecolor','r','markersize',3);
    plot(t_list,X_fit,'k','linewidth',2);
    title('Underdamped Fitting Tool Example');
    xlabel('Time (-)');
    ylabel('State (-)');
    mylegend = legend('Noisy Data','Underdamped Fit');
    set(mylegend,'location','northeast');
    
    %rescale y axis range a tad
    dX = max(X_list)-min(X_list);
    ylim([min(X_list),max(X_list)]+.1*dX*[-1,1]);
end