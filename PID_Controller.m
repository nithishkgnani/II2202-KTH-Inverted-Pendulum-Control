function U = PID_Controller(t, error, error_dot, gains)
% Simple PID: U = Kp*e + Ki*∫e dt + Kd*de/dt
% Call PID_Controller('reset') to clear persistent states.
persistent int_e prev_t prev_error
if ischar(t) && strcmpi(t,'reset')
    int_e = 0; prev_t = []; prev_error = 0;
    U = [];
    return;
end

if isempty(prev_t)
    dt = 0;
else
    dt = t - prev_t;
end

% Integrator
if isempty(int_e); int_e = 0; end
int_e = int_e + error * dt;

% Derivative: blend numerical and provided derivative
if dt > 0
    de_dt_num = (error - prev_error)/dt;
else
    de_dt_num = 0;
end
de_dt = 0.5*de_dt_num + 0.5*error_dot;

U = gains.Kp*error + gains.Ki*int_e + gains.Kd*de_dt;

prev_t = t;
prev_error = error;
end