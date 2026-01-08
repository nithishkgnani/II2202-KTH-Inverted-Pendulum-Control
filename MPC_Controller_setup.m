%setup mpc controller
function [mpcobj, xmpc, Ts, N_pred, N_ctrl] = MPC_Controller_setup(A, B, C, D)
            Ts = 0.01;        % sampling time [s]
            N_pred = 20;      % prediction horizon
            N_ctrl = 5;       % control horizon

            mpcobj = mpc(ss(A,B,C,D), Ts, N_pred, N_ctrl);

            % Weights
            mpcobj.Weights.OV = [1 0.1 10 0.1];  % output weights for [x, x_dot, theta, theta_dot]
            mpcobj.Weights.MV = 0.01;            % input weight
            mpcobj.Weights.MVRate = 0.01;        % optional: smooth input changes
            % Input constraints (motor voltage limits)
            mpcobj.MV = struct('Min',-6,'Max',6);
            xmpc = mpcstate(mpcobj);

end