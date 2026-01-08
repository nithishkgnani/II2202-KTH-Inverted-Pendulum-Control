function K = LQR_GainCalc(params,A,B)
%% Design LQR Controller
% Define weighting matrices Q and R
Q = diag([1, 0.1, 10, 0.1]); % Prioritize theta (angle) and x (position)
R = 0.01;                    % Penalize control effort

% Compute LQR gain
K = lqr(A, B, Q, R);
end