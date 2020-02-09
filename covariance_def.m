function [X_init, P_init, Q_wheel, Q_gamma, d_Maha_threshold] = covariance_def(X_start)
%Definition of initial esimation of robot pose & other necessary covariance
%matrix
% Output:
%   X_init [3x1] initial pose of robot
%   P_init [3x3] initial covariance of robot pose
%   Q_wheel [2x2] covariance of estimated wheel speed
%   Q_gamma [2x2] covariance of sensor noise
%   d_Maha_threshold (float) maximum value of Mahalanobis distance
%                            of a valid measurement

% Initial estimation of robot pose
X_init = X_start;
P_init = diag([0.2, 0.2, 0.1]);

% Covariance of wheel speed
sigma_tunning = 0.35;
Q_wheel = diag([sigma_tunning^2, sigma_tunning^2]);

% Covariance of sensor noise
sigma_alpha = 0.75;
sigma_r = 0.75;
Q_gamma = diag([sigma_alpha^2, sigma_r^2]);

% Mahalanobis distance threshold
d_Maha_threshold = 0.575;

end

