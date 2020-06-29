function [X_start_t, Z_t, sensor_state, u_mea] = env_step(X_start_tm1, twist, sensor_pos, t)
%Simulated environment
%Input:
%   X_start_tm1 [3 x 1]: last true robot pose
%   twist [2 x 1]: robot desired twist [v, omega]'
%   sensor_pos [2 x S]: sensor position in robot frame written in polar
%                       coordinate [rho, gamma]'
%                       Each column is the position of a sensor
%Output:
%   X_start_t [3 x 1]: current true robot pose [x, y, theta]'
%   Z_t [3 x K]: measurement vector. 
%               K is number of measurement        
%               Each column is a measurement [alpha, r, s] 
%               s is binary variable representing line type
%               s = 0 - vertical, s = 1 - horizontal
%   sensor_state [1xS] contains state of each sensor (0: not detect line, 1
%   detect line)
%   u_mea [2 x 1]: measured odometry [Delta D, Delta theta]'
%
% The measurement provided by the sensor is the coordinate of the detected
% line in Hough space in robot local frame.
% This file simulate the randomn effect of the environment on robot by
% adding zero-mean Gaussian noise to robot pose. In addition, the
% randomness in sensor behavior is replicated by perturbing sensor position
% with zero-mean Gaussian noise.


%% Constance
global tile_size num_edges edge_thick delta_t
env_wheel_sigma = 0.25;  % 0.025 true std deviation of zero-mean Gaussian noise added to wheel speed
% env_sensor_sigma = 0.0025;  % true std deviation of zero-mean Gaussian noise added to sensor position
% env_X_sigma_x = 0.055; % true std deviation of zero-mean Gaussian noise added to robot pose
% env_X_sigma_y = 0;
env_X_sigma_theta = 0.025;

% env_wheel_sigma = 0;  % true std deviation of zero-mean Gaussian noise added to wheel speed
env_sensor_sigma = 0;  % true std deviation of zero-mean Gaussian noise added to sensor position
env_X_sigma_x = 0; % true std deviation of zero-mean Gaussian noise added to robot pose
env_X_sigma_y = 0;
% env_X_sigma_theta = 0; 


%% Calculate current true pose
% calculate wheel speed
global e r_r r_l encoder_res
K = [r_r/2,     r_l/2;
     r_r/e,     -r_l/e];
q_dot = inv(K) * twist;
% perturb wheel speed
q_dot = q_dot +  normrnd([0, 0]', [env_wheel_sigma, env_wheel_sigma]');
% calculate amount of translation & rotation
u_t = delta_t * K * q_dot;
% Noise free advancement of robot pose
X_start_t = X_start_tm1 + [u_t(1) * cos(X_start_tm1(3));
                           u_t(1) * sin(X_start_tm1(3));
                           u_t(2)];
% Perturb X_start_t
X_start_t = X_start_t + normrnd([0, 0, 0]', [env_X_sigma_x, env_X_sigma_y, env_X_sigma_theta]');
                       
%% Simulate encoder
encoder_counts = floor(delta_t * q_dot / (2 * pi / encoder_res));
delta_q_mea = encoder_counts * (2 * pi / encoder_res);
u_mea = K * delta_q_mea;
                       
%% Calculate measurement vector
Z_t = [];  % initialize measurement vector
sensor_state = zeros(1, size(sensor_pos, 2));  % initialize sensor state

% Find the tile that contains robot
[tile_x, tile_y] = tile_localize(X_start_t, tile_size, num_edges);

for s = 1 : size(sensor_pos, 2)
    % Find sensor position in global frame
    rho = sensor_pos(1, s);
    gamma = sensor_pos(2, s);
    s_pos = X_start_t(1 : 2) + rot_z(X_start_t(3)) * rho * [cos(gamma);
                                                            sin(gamma)];
    % Perturb sensor position (to simulate sensor's uncertainty)
    s_pos = s_pos + normrnd(0, env_sensor_sigma, [2, 1]);
    % Check if robot is close enough to either left or right edge of the tile to detect it
    if abs(tile_x * tile_size - s_pos(1)) <= edge_thick || abs((tile_x + 1) * tile_size - s_pos(1)) <= edge_thick
        % calculate measurement for a vertical line
        z = [-X_start_t(3);
             abs(rho * cos(X_start_t(3) + gamma));  % just to simulate the output of the sensor
             0; % 0 for vertical
             s; % sensor number
             t]; %timestep
        % store this measurement 
        Z_t= [Z_t, z]; 
        % update sensor state
        sensor_state(s) = 1;
    end
    % Check if robot is close enough to either lower or upper edge of the tile to detect it
    if abs(tile_y * tile_size - s_pos(2)) <= edge_thick || abs((tile_y + 1) * tile_size - s_pos(2)) <= edge_thick
        % calculate measurement for a horizontal line
        z = [pi/2 - X_start_t(3);
             abs(rho * sin(X_start_t(3) + gamma));
             1; % 1 for horizontal
             s; % sensor number
             t]; % timestep
        % store this measurement 
        Z_t= [Z_t, z]; 
        % update sensor state
        sensor_state(s) = 1;
    end
end

end

