function [mu_bar, sigma_bar] = prediction_step(mu, sigma, u)
%Prediction step of the Kalman filter
%Input:
%   mu [3x1]: mean of robot pose [x, y, theta]'
%   sigma [3x3]: covariant of robot pose
%   u [2x1]: odometry [Delta D, Delta theta]'
%Output:
%   mu_bar [3x1]: mean of robot pose esitmated by motion model
%   sigma_bar [3x3]: esitmated covariant matrix of robot pose 


% Estimated position
mu_bar = mu + [u(1) * cos(mu(3));
               u(1) * sin(mu(3));
               u(2)];
           
% Jacobian of motion model
Jx = [1, 0, -u(1) * sin(mu(3));
      0, 1, u(1) * cos(mu(3));
      0, 0, 1];

Ju = [cos(mu(3)), 0;
      sin(mu(3)), 0;
      0, 1];


% Covariance of control signal (i.e. the odometry)
r_r = 0.1;  % right wheel radius
r_l = 0.1;  % left wheel radius
e = 0.2;  % track gauge
K = [r_r/2, r_l/2;
    r_r/e, -r_l/e];
sigma_wheel = 0.25;  % need to tune
sigma_u = K * diag([sigma_wheel^2, sigma_wheel^2]) * K';

% Covariance of estimated robot pose
sigma_bar = Jx * sigma * Jx' + Ju * sigma_u * Ju';

end

