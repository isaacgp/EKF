function [X_t, P_t] = ekf_estimation(X_tm1, P_tm1, u_t, Z_t)
%Estimate robot's current pose with EKF
% Input:
%   X_tm1 [3x1] last robot pose [x, y, theta]'
%   P_tm1 [3x3] last covariance of robot pose
%   u_t [2x1] current odometry [\Delta D, \Delta theta]
%   Z_t [3xK] measurement vector. 
%               K is number of measurement        
%               Each column is a measurement [alpha, r, s] 
%               s is binary variable representing line type
%               s = 0 - vertical, s = 1 - horizontal

global Q_gamma d_Maha_threshold

% Estimation using motion model
[X_bar, P_bar] = motion_model(X_tm1, P_tm1, u_t);

% Data association
[V, PV, C_tensor] = data_associate(X_bar, P_bar, Z_t, Q_gamma, d_Maha_threshold);

% Correction
[X_t, P_t] = correction_step(X_bar, P_bar, V, PV, C_tensor);

end

