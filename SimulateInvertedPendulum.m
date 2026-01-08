clear all;
clc
close all;

%% MATLAB implementation of Sigi plant model
% ===== Parameters =====
% Motor/gearbox
params.K_m = (6.9401e-04+7.4663e-04)/2;     % Motor constant [Nm/A]
params.R_m = (6.1298+6.7397)/2;             % Motor resistance [Ω]
params.i_gb = 30;                           % Gearbox ratio
params.r = 0.04;                            % Wheel radius [m]

% Mass properties
params.m_B = 0.368;    % Body mass [kg]
clear all;
clc
close all;

%% MATLAB implementation of Sigi plant model
% ===== Parameters =====
% Motor/gearbox
params.K_m = (6.9401e-04+7.4663e-04)/2;     % Motor constant [Nm/A]
params.R_m = (6.1298+6.7397)/2;             % Motor resistance [Ω]
params.i_gb = 30;                           % Gearbox ratio
params.r = 0.04;                            % Wheel radius [m]

% Mass properties
params.m_B = 0.368;    % Body mass [kg]
params.m_W = 0.02;     % Wheel mass [kg]
params.l = 0.01;       % Distance axle -> CoM [m]
params.J = 2.249*10^-5;   % Wheel inertia [kg·m²]
params.g = 9.81;       % Gravity [m/s²]
params.I_2 = 2.175e-4; % Body inertia
params.d_ff = 0.00;    % Fluid friction coefficient
% Motor saturation (common for all controllers)
params.Umax = 6;      % [V]

% Plotting Mode
plotMode = 'combined'; % Options: 'combined', 'separate'



% Simulation setup
tspan = [0 15];

% System linearization
[A, B, C, D] = LinearizeSystem(params);

% ---- LQR Gain (for reference) ----
K_LQR = LQR_GainCalc(params,A,B);

% ---- PID Gains (simple theta + position) ----
pidParams.theta.Kp = -60;
pidParams.theta.Ki = -5;
pidParams.theta.Kd = -2;
pidParams.x.Kp     = -1.5;
pidParams.x.Ki     = -0.2;
pidParams.x.Kd     = -0.05;
% Use common motor saturation
pidParams.Umax     = params.Umax;

%Simulation parameters for MPC
%% 4. Simulation parameters
Tsim = 15;           % total simulation time [s]
x0_mpc = [0; 0; 0; 0]; % initial state for MPC setup (dummy)
ref = [0;0;0;0];     % reference state (upright, centered)
[mpcobj, xmpc, Ts, N_pred, N_ctrl] = MPC_Controller_setup(A,B,C,D);
% Update MPC constraints to match params.Umax
mpcobj.MV.Min = -params.Umax;
mpcobj.MV.Max = params.Umax;


%% Define Experiments
experiments(1).name = 'Experiment A: Nominal Stabilization';
experiments(1).theta_init = 5.0;
experiments(1).dist_force = 0;
experiments(1).dist_time = 0;
experiments(1).description = 'Initial Angle 5 deg, No Disturbance';

experiments(2).name = 'Experiment B: Constraint Handling';
experiments(2).theta_init = 20.0; % Increased angle to hit saturation
experiments(2).dist_force = 0;
experiments(2).dist_time = 0;
experiments(2).description = 'Initial Angle 20 deg, No Disturbance';

experiments(3).name = 'Experiment C: Disturbance Rejection';
experiments(3).theta_init = 0.0;
experiments(3).dist_force = 1.0; % N (Impulse magnitude)
experiments(3).dist_time = 3.0;  % s
experiments(3).description = 'Initial Angle 0 deg, Disturbance at t=3s';

controllers = {'LQR', 'MPC'}; % {'PID', 'LQR', 'MPC'}
results = struct();

% Which controllers to plot when in 'combined' mode.
% Options:
% - leave as empty array [] to use all controllers
% - set to a cell array, e.g. {'PID','LQR'} to manually select
% - set useInteractiveSelection = true to prompt a dialog (if running
%   in interactive MATLAB)
plotControllers = []; %{'LQR', 'MPC'}; % e.g. {'PID','LQR'} or [] to use all
useInteractiveSelection = false; % set true to open a selection dialog

% Resolve selection (now that `controllers` is defined)
if isempty(plotControllers)
    plotControllers = controllers;
end
if useInteractiveSelection
    try
        idx = listdlg('ListString', controllers, 'SelectionMode', 'multiple', 'Name', 'Select controllers to plot', 'PromptString', 'Select controllers to include in combined plot:');
        if ~isempty(idx)
            plotControllers = controllers(idx);
        end
    catch
        % listdlg may not be available in non-interactive environments; ignore
    end
end

% Plot font size (adjust to taste)
plotFontSize = 14; % e.g. 12-18 for presentations

%% Run Experiments
for i = 1:length(experiments)
    exp = experiments(i);
    fprintf('\n========================================\n');
    fprintf('Running %s\n', exp.name);
    fprintf('%s\n', exp.description);
    fprintf('========================================\n');
    
    for c = 1:length(controllers)
        ctrl = controllers{c};
        fprintf('  Simulating %s...\n', ctrl);
        
        % Simulate
        [t, X, U] = simulate_plant(ctrl, tspan, exp.theta_init, params, K_LQR, pidParams, mpcobj, xmpc, ref, exp.dist_force, exp.dist_time);
        
        % Store
        results(i).(ctrl).t = t;
        results(i).(ctrl).X = X;
        results(i).(ctrl).U = U;
        
        % Calculate Metrics (Theta)
        step_time = exp.dist_time; 
        if step_time == 0 && exp.theta_init ~= 0
             step_time = 0;
        end
        
        metrics = calculate_metrics(t, rad2deg(X(:,3)), 0, step_time);
        results(i).(ctrl).metrics = metrics;
        
        fprintf('    Settling Time: %.4f s\n', metrics.SettlingTime);
        fprintf('    Peak Amplitude: %.4f deg\n', metrics.PeakAmplitude);
    end
    
    % Plot Comparison or Separate
    if strcmp(plotMode, 'combined')
        % Use user-selected controllers for the combined plot
        plot_experiment_comparison(results(i), exp.name, plotControllers, params.Umax);
    else
        for c = 1:length(controllers)
            ctrl = controllers{c};
            plot_single_result(results(i).(ctrl), [exp.name ' - ' ctrl], params.Umax);
        end
    end
end

%% Save Results
save('simulation_results2.mat', 'results', 'params', 'experiments');
fprintf('\nResults saved to simulation_results.mat\n');

%% Functions
function [t, X, U_out] = simulate_plant(controllerType, tspan, theta_init_deg, params, K_LQR, pidParams, mpcobj, xmpc, ref, dist_force, dist_time)
    a = params.m_B * params.l;
    I_O = params.I_2 + params.m_B*params.l^2;
    m_tot = params.m_B + 2*params.m_W;
    m_O = m_tot + params.J/params.r^2;

    x0 = [0; 0; deg2rad(theta_init_deg); 0]; % [x; x_dot; theta; theta_dot]

    % Define time segments for disturbance
    if dist_force ~= 0
        % Segment 1: Start to Disturbance Start
        t1 = [tspan(1), dist_time];
        % Segment 2: Disturbance Active (0.1s duration)
        t2 = [dist_time, dist_time + 0.1];
        % Segment 3: Post Disturbance
        t3 = [dist_time + 0.1, tspan(2)];
        segments = {t1, t2, t3};
        forces   = [0, dist_force, 0];
    else
        segments = {tspan};
        forces = [0];
    end
    
    t = [];
    X = [];
    U_out = [];
    
    % For MPC, we need to maintain state across segments
    current_x = x0;
    current_xmpc = [];
    if strcmp(upper(controllerType), 'MPC')
        current_xmpc = mpcstate(mpcobj);
    end

    for s = 1:length(segments)
        seg_tspan = segments{s};
        seg_force = forces(s);
        
        if seg_tspan(2) <= seg_tspan(1)
            continue; 
        end
        
        switch upper(controllerType)
            case 'PID'
                if s==1, PID_Controller('reset'); end
                dynFun = @(t,x) plant_dynamics_PID(t,x,params,a,I_O,m_O,pidParams, seg_force);
                [t_seg, X_seg] = ode45(dynFun, seg_tspan, current_x);
                
                % Reconstruct U
                U_seg = zeros(length(t_seg), 1);
                for k=1:length(t_seg)
                    [~, u_val] = plant_dynamics_PID(t_seg(k), X_seg(k,:)', params, a, I_O, m_O, pidParams, seg_force);
                    U_seg(k) = u_val;
                end
                
            case 'LQR'
                dynFun = @(t,x) plant_dynamics_LQR(t,x,params,a,I_O,m_O,K_LQR, seg_force);
                [t_seg, X_seg] = ode45(dynFun, seg_tspan, current_x);
                
                U_seg = zeros(length(t_seg), 1);
                for k=1:length(t_seg)
                    [~, u_val] = plant_dynamics_LQR(t_seg(k), X_seg(k,:)', params, a, I_O, m_O, K_LQR, seg_force);
                    U_seg(k) = u_val;
                end
                
            case 'MPC'
                % MPC handles its own stepping
                % We need to run the MPC loop for the duration of this segment
                Ts = mpcobj.Ts;
                Nsteps = ceil((seg_tspan(2) - seg_tspan(1))/Ts);
                
                t_seg = [];
                X_seg = [];
                U_seg = [];
                
                local_x = current_x;
                t0_local = seg_tspan(1);
                
                for k = 1:Nsteps
                    y_meas = local_x'; 
                    [u, Info] = mpcmove(mpcobj, current_xmpc, y_meas, ref');
                    
                    t_next = min(t0_local + Ts, seg_tspan(2));
                    t_span_step = [t0_local, t_next];
                    
                    dyn_with_u = @(t,x) plant_dynamics_MPC(t, x, params, a, I_O, m_O, u, seg_force);
                    [t_step, X_step] = ode45(dyn_with_u, t_span_step, local_x);
                    
                    if isempty(t_seg)
                        t_seg = t_step;
                        X_seg = X_step;
                        U_seg = repmat(u, length(t_step), 1);
                    else
                        t_seg = [t_seg; t_step(2:end)];
                        X_seg = [X_seg; X_step(2:end,:)];
                        U_seg = [U_seg; repmat(u, length(t_step)-1, 1)];
                    end
                    
                    local_x = X_step(end,:)';
                    t0_local = t_step(end);
                    
                    if t0_local >= seg_tspan(2) - 1e-10
                        break;
                    end
                end
                current_x = local_x; % Update for next segment
                
        end
        
        % Append to main history
        if isempty(t)
            t = t_seg;
            X = X_seg;
            U_out = U_seg;
        else
            % Avoid duplicate point
            t = [t; t_seg(2:end)];
            X = [X; X_seg(2:end,:)];
            U_out = [U_out; U_seg(2:end)];
        end
        
        if ~strcmp(upper(controllerType), 'MPC')
            current_x = X_seg(end,:)';
        end
    end

end

function [dxdt, U] = plant_dynamics_LQR(t,x,params,a,I_O,m_O,K, dist_force)
    x_pos     = x(1);
    x_dot     = x(2);
    theta     = x(3);
    theta_dot = x(4);
    
    % LQR state-feedback law (saturated)
    U_raw = -K * [x_pos; x_dot; theta; theta_dot];
    U = max(min(U_raw, params.Umax), -params.Umax);
    
    dxdt = compute_accelerations(x, U, params, a, I_O, m_O, t, dist_force);
end

function [dxdt, U] = plant_dynamics_MPC(t,x,params,a,I_O,m_O,u_mpc, dist_force)
    % MPC-driven plant dynamics: treat u_mpc as constant voltage
    U = max(min(u_mpc, params.Umax), -params.Umax);
    dxdt = compute_accelerations(x, U, params, a, I_O, m_O, t, dist_force);
end

function [dxdt, U] = plant_dynamics_PID(t,x,params,a,I_O,m_O,pidParams, dist_force)
    x_pos     = x(1);
    x_dot     = x(2);
    theta     = x(3);
    theta_dot = x(4);

    % Errors (setpoints zero)
    e_theta = -theta;
    e_x     = -x_pos;

    % PID law (voltage)
    % FIX: Pass -theta_dot because error_dot = d(-theta)/dt = -theta_dot
    U_theta = PID_Controller(t, e_theta, -theta_dot, pidParams.theta);
    U_x     = PID_Controller(t, e_x, x_dot,    pidParams.x); % x_dot is correct? e_x = -x. de/dt = -x_dot. Wait.

    % Check x sign:
    % e_x = -x.
    % de_x/dt = -x_dot.
    % We passed x_dot.
    % So we should pass -x_dot to PID_Controller for x as well.
    U_x     = PID_Controller(t, e_x, -x_dot,    pidParams.x);

    U_raw = U_theta + U_x;
    U = max(min(U_raw, pidParams.Umax), -pidParams.Umax);

    dxdt = compute_accelerations(x, U, params, a, I_O, m_O, t, dist_force);
end

function dxdt = compute_accelerations(x, U, params, a, I_O, m_O, t, dist_force)
    x_dot     = x(2);
    theta     = x(3);
    theta_dot = x(4);
    
    x_w_dot = x_dot/params.r;
    
    % Corrected T_m with i_gb
    T_m = (params.i_gb*params.K_m/params.R_m)*U - (params.i_gb^2*params.K_m^2/params.R_m)*x_w_dot + (params.i_gb^2*params.K_m^2/params.R_m)*theta_dot;

    d1 = I_O*m_O - (a*cos(theta))^2;
    if abs(d1) < 1e-6
        error('Singularity in d1 denominator');
    end

    x_ddot = (1/d1)*( a*I_O*theta_dot^2*sin(theta) ...
                    - a^2*params.g*sin(theta)*cos(theta) ...
                    + T_m*(I_O/params.r + a*cos(theta)) ...
                    + I_O * dist_force ); 
                    
    x_ddot = x_ddot - params.d_ff*x_dot;

    theta_ddot = (1/d1)*( -a^2*theta_dot^2*sin(theta)*cos(theta) ...
                        + a*m_O*params.g*sin(theta) ...
                        - T_m*(m_O + (a/params.r)*cos(theta)) ...
                        - a*cos(theta) * dist_force );

    dxdt = [x_dot;
            x_ddot;
            theta_dot;
            theta_ddot];
end

function plot_experiment_comparison(res, expName, controllers, Umax)
    % Combined plot: 2x2 for angle/vel and position/vel, control input in separate figure
    figure('Name', expName, 'Color', 'w', 'Position', [100, 100, 1000, 500]);

    % Use default font size if not provided
    if exist('plotFontSize','var') ~= 1
        plotFontSize = 14;
    end

    nCtrl = length(controllers);
    colors = lines(max(1,nCtrl));
    lineStyles = {'-','--','-.',':'};

    % Layout: 2x2 grid for angles and linear states
    % (1,1) Angle (theta)
    subplot(2,2,1); hold on;
    for c = 1:nCtrl
        ctrl = controllers{c};
        plot(res.(ctrl).t, rad2deg(res.(ctrl).X(:,3)), 'Color', colors(c,:), 'LineStyle', lineStyles{mod(c-1,length(lineStyles))+1}, 'LineWidth', 1.5);
    end
    ylabel('Theta [deg]','FontSize',plotFontSize);
    xlabel('Time [s]','FontSize',plotFontSize);
    title('Angle','FontSize',plotFontSize);
    grid on;
    set(gca,'FontSize',plotFontSize);
    legend(controllers, 'Interpreter', 'none', 'Location', 'best', 'FontSize', max(8,plotFontSize-2));

    % (1,2) Angular velocity (theta_dot)
    subplot(2,2,2); hold on;
    for c = 1:nCtrl
        ctrl = controllers{c};
        plot(res.(ctrl).t, rad2deg(res.(ctrl).X(:,4)), 'Color', colors(c,:), 'LineStyle', lineStyles{mod(c-1,length(lineStyles))+1}, 'LineWidth', 1.2);
    end
    ylabel('Theta dot [deg/s]','FontSize',plotFontSize);
    xlabel('Time [s]','FontSize',plotFontSize);
    title('Angular velocity','FontSize',plotFontSize);
    grid on;
    set(gca,'FontSize',plotFontSize);

    % (2,1) Linear position (x)
    subplot(2,2,3); hold on;
    for c = 1:nCtrl
        ctrl = controllers{c};
        plot(res.(ctrl).t, res.(ctrl).X(:,1), 'Color', colors(c,:), 'LineStyle', lineStyles{mod(c-1,length(lineStyles))+1}, 'LineWidth', 1.2);
    end
    ylabel('Position [m]','FontSize',plotFontSize);
    xlabel('Time [s]','FontSize',plotFontSize);
    title('Position','FontSize',plotFontSize);
    grid on;
    set(gca,'FontSize',plotFontSize);

    % (2,2) Linear velocity (x_dot)
    subplot(2,2,4); hold on;
    for c = 1:nCtrl
        ctrl = controllers{c};
        plot(res.(ctrl).t, res.(ctrl).X(:,2), 'Color', colors(c,:), 'LineStyle', lineStyles{mod(c-1,length(lineStyles))+1}, 'LineWidth', 1.2);
    end
    ylabel('Velocity [m/s]','FontSize',plotFontSize);
    xlabel('Time [s]','FontSize',plotFontSize);
    title('Velocity','FontSize',plotFontSize);
    grid on;
    set(gca,'FontSize',plotFontSize);

    % % Overall title
    % h = sgtitle(expName);
    % if ~isempty(h)
    %     set(h,'FontSize',plotFontSize+2);
    % end

    % Save States Figure
    cleanName = regexprep(expName, '[^a-zA-Z0-9]', '_');
    cleanName = regexprep(cleanName, '_+', '_');
    % Remove leading/trailing underscores
    cleanName = regexprep(cleanName, '^_|_$', '');
    saveas(gcf, [cleanName '_States.png']);

    % Separate figure for control input
    figure('Name', [expName ' - Control Input'], 'Color', 'w', 'Position', [100, 100, 1000, 500]); hold on;
    for c = 1:nCtrl
        ctrl = controllers{c};
        plot(res.(ctrl).t, res.(ctrl).U, 'Color', colors(c,:), 'LineStyle', lineStyles{mod(c-1,length(lineStyles))+1}, 'LineWidth', 1.5);
    end
    yline(Umax, 'k--');
    yline(-Umax, 'k--');
    ylabel('Voltage [V]','FontSize',plotFontSize);
    xlabel('Time [s]','FontSize',plotFontSize);
    % title('Control Input Voltage','FontSize',plotFontSize);
    grid on;
    set(gca,'FontSize',plotFontSize);
    legend(controllers, 'Interpreter', 'none', 'Location', 'best', 'FontSize', max(8,plotFontSize-2));
    
    % Save Input Figure
    saveas(gcf, [cleanName '_Input.png']);
end

function plot_single_result(res, titleStr, Umax)
    figure('Name', titleStr, 'Color', 'w', 'Position', [100, 100, 1000, 500]);
    % Use default font size if not provided
    if exist('plotFontSize','var') ~= 1
        plotFontSize = 14;
    end

    subplot(2,2,1);
    plot(res.t, res.X(:,1), 'b', 'LineWidth', 1.5);
    ylabel('Position [m]','FontSize',plotFontSize); grid on;
    title('Position','FontSize',plotFontSize);
    set(gca,'FontSize',plotFontSize);

    subplot(2,2,2);
    plot(res.t, res.X(:,2), 'r', 'LineWidth', 1.5);
    ylabel('Velocity [m/s]','FontSize',plotFontSize); grid on;
    title('Velocity','FontSize',plotFontSize);
    set(gca,'FontSize',plotFontSize);

    subplot(2,2,3);
    plot(res.t, rad2deg(res.X(:,3)), 'g', 'LineWidth', 1.5);
    ylabel('Theta [deg]','FontSize',plotFontSize); xlabel('Time [s]','FontSize',plotFontSize); grid on;
    title('Angle','FontSize',plotFontSize);
    set(gca,'FontSize',plotFontSize);

    subplot(2,2,4);
    plot(res.t, res.U, 'm', 'LineWidth', 1.5);
    yline(Umax, 'k--'); yline(-Umax, 'k--');
    ylabel('Voltage [V]','FontSize',plotFontSize); xlabel('Time [s]','FontSize',plotFontSize); grid on;
    title('Control Input','FontSize',plotFontSize);
    set(gca,'FontSize',plotFontSize);

    h = sgtitle(titleStr);
    if ~isempty(h)
        set(h,'FontSize',plotFontSize+2);
    end

    % Save Figure
    cleanName = regexprep(titleStr, '[^a-zA-Z0-9]', '_');
    cleanName = regexprep(cleanName, '_+', '_');
    % Remove leading/trailing underscores
    cleanName = regexprep(cleanName, '^_|_$', '');
    saveas(gcf, [cleanName '.png']);
end