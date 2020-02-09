function [V, PV, C_tensor] = data_associate(X_bar, P_bar, Z_t, Q_gamma, dMaha_threshold)
%Associate the measurement obtained by the sensor and landmarks (lines) in
%the map by thresholding the Mahalanobis distance of the innovation vector.
% Input:
%   X_bar [3 x 1] robot pose estimated by motion model
%   P_bar [3 x 3] covariance of robot pose
%   Z_t [3 x K] sensor output
%   Q_gamma [2 x 2] covariance of sensor noise
%   dMaha_threshold (float) maximum Mahalanobis distance of a valid measurement 
% Output:
%   V [2 x num_v] collection of innovation vector of associated measurements
%   PV [2 x 2 x num_v] covariance of innovation vector of associated measurements 
%   C_tensor [2 x 3 x num_v] collection of Jacobian of measurement model of associated measurements

global tile_size num_edges

% Initialize 
V = [];
num_v = 0;  % number of associated measurement
PV = [];
C_tensor = [];

% Find the tile that contains robot
[tile_x, tile_y] = tile_localize(X_bar, tile_size, num_edges);

% Iterate through all the measurement and try to associate each of them 
% with an edge
for i = 1 : size(Z_t, 2)
    % Extract true measurement
    z = Z_t(:, i);
    % Find which edge of the tile is actually measured
%     if not(isempty(z)) &&  z(3) == 0  
    if z(3) == 0 
        % Detect a vertical line  
        [v_l, P_v_l, C_l, d_maha_l] = cal_innovation(tile_x, z, X_bar, P_bar, Q_gamma);
        [v_r, P_v_r, C_r, d_maha_r] = cal_innovation(tile_x + 1, z, X_bar, P_bar, Q_gamma);
        if min([d_maha_l, d_maha_r]) < dMaha_threshold^2
            if d_maha_l < d_maha_r
                % inner edge is detected
                V = [V, v_l]; 
                num_v = num_v + 1;
                PV(:, :, num_v) = P_v_l;
                C_tensor(:, :, num_v) = C_l;
            else
                % outter edge is detected
                V = [V, v_r];
                num_v = num_v + 1;
                PV(:,:, num_v) = P_v_r;
                C_tensor(:, :, num_v) = C_r;
            end
        end
%     elseif not(isempty(z)) && z(3) == 1 
     elseif z(3) == 1 
        % Detect a horizontal line
        [v_d, P_v_d, C_d, d_maha_d] = cal_innovation(tile_y, z, X_bar, P_bar, Q_gamma);
        [v_u, P_v_u, C_u, d_maha_u] = cal_innovation(tile_y + 1, z, X_bar, P_bar, Q_gamma);
        if min([d_maha_d, d_maha_u]) < dMaha_threshold^2
            if d_maha_d < d_maha_u
                % lower edge is detected
                V = [V, v_d]; 
                num_v = num_v + 1;
                PV(:, :, num_v) = P_v_d;
                C_tensor(:, :, num_v) = C_d;
            else
                % upper edge is detected
                V = [V, v_u];
                num_v = num_v + 1;
                PV(:,:, num_v) = P_v_u;
                C_tensor(:, :, num_v) = C_u;
            end
        end
    end
end

end

