function [A, B, C, D] = LinearizeSystem(params)
% This function linearizes the inverted pendulum system dynamics around the equilibrium point (theta=0, x_dot=0, theta_dot=0).

    % Derived parameters
    a = params.m_B * params.l;
    % I_2 = 2.175*10^-4; % Body inertia
    I_O = params.I_2 + params.m_B*params.l^2;
    m_tot = params.m_B + 2*params.m_W;
    m_O = m_tot + params.J/params.r^2;

    % Define symbolic variables
    syms x x_dot theta theta_dot U real;

    % Motor torque equation
    x_w_dot = x_dot / params.r;
    T_m = (params.i_gb*params.K_m/params.R_m)*U - (params.i_gb^2*params.K_m^2/params.R_m)*x_w_dot + (params.i_gb^2*params.K_m^2/params.R_m)*theta_dot;

    % Compute d1
    d1 = I_O*m_O - (a*cos(theta))^2;

    % Nonlinear dynamics (x_ddot and theta_ddot from Equations 21–22)
    x_ddot = (1/d1) * (a*I_O*theta_dot^2*sin(theta) - a^2*params.g*sin(theta)*cos(theta) + T_m*(I_O/params.r + a*cos(theta)));
    theta_ddot = (1/d1) * (-a^2*theta_dot^2*sin(theta)*cos(theta) + a*m_O*params.g*sin(theta) - T_m*(m_O + (a/params.r)*cos(theta)));

    % State vector derivatives
    f = [x_dot; x_ddot; theta_dot; theta_ddot];

    % Linearize the System
    eq_point = [x, 0, 0, 0]; % theta=0, x_dot=0, theta_dot=0
    A = subs(jacobian(f, [x, x_dot, theta, theta_dot]), [x_dot, theta, theta_dot], [0, 0, 0]);
    B = subs(jacobian(f, U), [x_dot, theta, theta_dot], [0, 0, 0]);

    % Convert symbolic matrices to numeric
    A = double(A);
    B = double(B);
    C = eye(4); 
    D = zeros(4,1);

end