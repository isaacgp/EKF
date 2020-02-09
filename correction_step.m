function [X_t, P_t] = correction_step(X_bar, P_bar, V, PV, C_tensor)
%Correction step of EKF Localization
%Input:
%   X_bar[3x1] estimated robot pose (using motion model only)
%   P_bar [3x3] estimated covariance
%   V [2 x num_v] collection of innovation vector of associated measurements
%   PV [2 x 2 x num_v] covariance of innovation vector of associated measurements
%   C_tensor [2 x 3 x num_v] collection of Jacobian of measurement model of associated measurements 
%Output
%   X_t [3x1]: corrected pose
%   P_t [3x3]: corrected covariance

X_t = X_bar;
P_t = P_bar;

if V  % there are measurements associated with environment edges
    for i = 1 : size(V, 2)
        % extract innovation vector, its covariance & Jacobian of measurement model
        v = V(:, i); 
        P_v = PV(:, :, i);
        C = C_tensor(:, :, i);
        % Kalman gain
        K = P_t * C' / P_v;
        % Correction
        X_t = X_t + K * v;
        P_t = (eye(3) - K * C) * P_t;
    end
% else
%     disp("No correction here!")
end


end