function [v, P_v, C, d_maha] = cal_innovation(edge_idx, z, X_bar, P_bar, Q_gamma)
% Calculate innovation for a given edge
% Input:
%   edge_idx (int) index of the edge which is being considered
%   z [3x1] actual sensor measurement
%   X_bar [3x1] robot pose estimated by motion model
%   P_bar [3x3] covariance of robot pose
%   Q_gamma [2x2] covariance of sensor noise
% Output:
%   v [2x1] innovation vector (the last entry of z is a binary variable to 
%   signify the type of detected edge. The measurement is just contained
%   in the first 2 elemenents of z, hence size of v is 2)
%   P_v [2x2] covariance of innovation vecotr
%   C [2x3] Jacobian of measurement model
%   d_maha (float) Mahalanobis distance of this edge

global tile_size

% Expected measurement
if z(3) == 0
    % detect vertical line
    z_hat = [-X_bar(3);
             abs(edge_idx * tile_size - X_bar(1))];  % measurement model
    % Jacobian of measurement model
    if edge_idx * tile_size > X_bar(1)
        C = [0,     0,      -1;
             -1,    0,      0]; %right line
    else
        C = [0,     0,      -1;
             1,     0,      0]; %left line
    end
else
    % detect a horizontal line
    z_hat = [pi/2 - X_bar(3);
             abs(edge_idx * tile_size - X_bar(2))];
    % Jacobian of measurement model
    if edge_idx * tile_size > X_bar(2)
        C = [0,     0,      -1; % upper line
             0,     -1,     0];
    else
        C = [0,     0,      -1; %lower line
             0,     1,      0];
    end
end

% Innovation vector
v = z(1:2) - z_hat;

% Covariance of innovation vector
P_v = Q_gamma + C * P_bar * C';

% Mahalanobis distance
% d_maha =  v' * inv(P_v) * v;
d_maha = sqrt( v' * inv(P_v) * v);
     
end