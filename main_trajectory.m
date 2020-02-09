function [simulated_data, v_history] = main_trajectory()

%EKF Localization main file

% clear 
% close all
rng('default') % For reproducibility
% clc

display_sensor = false;  % change to true to display sensor state

global tile_size num_edges edge_thick Q_gamma d_Maha_threshold Q_wheel e r_r r_l delta_t encoder_res

% Environment configuration
tile_size = 1.;
edge_thick = 2 * 0.05;
num_edges = 11;  % edges are indexed from 0 (starting from y-axis for vertical edge)
delta_t = 0.3;  % sampling time

% Robot configuration
[e, r_r, r_l, sensor_pos, encoder_res] = robot_def(); 

% Define waypoints
wp = [1.5, 2.5, 3.5, 5, 7;   % x-coordinate of waypoints
      3.2, 3.9, 5, 6, 4.5 ];  % y-coordinate of waypoints
last_wp_idx = 1;
                                  
% Initialize true robot pose
X_start = [0.1, 2.2, 0]'; 

% Get initial estimation of robot state & necessary covariance matrices
[X_start, P, Q_wheel, Q_gamma, d_Maha_threshold] = covariance_def(X_start);

last_t = 200;  % final time step
sensor_history = zeros(last_t, size(sensor_pos, 2));
X_start_history = zeros(last_t, 3);
v_history = zeros(last_t,2);
% simulated_data = zeros(1,3);


x_true = [];
x_upper = [];
x_lower = [];
y_true = [];
y_upper = [];
y_lower = [];
theta_true = [];
theta_upper = [];
theta_lower = [];
x_odom = [X_start(1)];
y_odom = [X_start(2)];
theta_odom = [X_start(3)];
sigma_x = [];
sigma_y = [];
sigma_theta = [];
detect_line_timestep = [];

for t = 1 : last_t
    % Choose waypoints
    if norm(X_start(1:2) - wp(:, last_wp_idx)) < 0.5
        last_wp_idx = last_wp_idx + 1;
    end
    
    if last_wp_idx > length(wp)  % travered all waypoints
        break 
    end
    
    % Calculate desired twist to get to desired waypoint
    [v, omega] = compute_velocity(wp(:, last_wp_idx), X_start);
%     v_history = [v_history;v];
    v_history = [v_history;v,omega];
    
    % Query the environment for new pose, measurement, & odometry
    [X_start, Z, sensor_state, u] = env_step(X_start, [v, omega]', sensor_pos, t);
%     [m,n] = size(Z);
%     if t==1
%         if n==0
%             simulated_data = [X_start',Z, sensor_state, u'];
%         elseif n==1
%             simulated_data = [X_start',Z', 0, 0, -1, 0, 0, -1, sensor_state, u'];
%         elseif n==2
%             simulated_data = [X_start',Z(:,1)',Z(:,2)', 0, 0, -1, sensor_state, u'];
%         else
%             simulated_data = [X_start',Z(:,1)',Z(:,2)',Z(:,3)', sensor_state, u'];
%         end
%     else
%         if n==0
%             simulated_data = [simulated_data;X_start',0, 0, -1, 0, 0, -1, 0, 0, -1, sensor_state, u'];
%         elseif n==1
%             simulated_data = [simulated_data;X_start',Z', 0, 0, -1, 0, 0, -1, sensor_state, u'];
%         elseif n==2
%             simulated_data = [simulated_data;X_start',Z(:,1)',Z(:,2)', 0, 0, -1, sensor_state, u'];
%         else
%             simulated_data = [simulated_data;X_start',Z(:,1)',Z(:,2)',Z(:,3)', sensor_state, u'];
%         end
% 
%         simulated_data = [simulated_data;X_start',Z, sensor_state, u'];
if t==1
%     csvwrite('simulated_data.txt',[X_start',Z, sensor_state, u'],0,0)
    csvwrite('simulated_data.txt',[X_start', sensor_state, u'],0,0)
    type simulated_data.txt
else
    switch size(Z,2)
        case 0
            dlmwrite('simulated_data.txt',[X_start', sensor_state, u'],'-append');
        case 1
            dlmwrite('simulated_data.txt',[X_start', sensor_state, u', Z'],'-append');
        case 2
            dlmwrite('simulated_data.txt',[X_start', sensor_state, u', Z(:,1)', Z(:,2)'],'-append');
        case 3
            dlmwrite('simulated_data.txt',[X_start', sensor_state, u', Z(:,1)', Z(:,2)', Z(:,3)'],'-append');
    end
end


%         [X_start, Z, sensor_state, u] = env_step(simulated_data(t,:)', [v, omega]', sensor_pos);
    %end
    
    % Compute odometry estimation
    x_odom = [x_odom, x_odom(end) + u(1) * cos(theta_odom(end))];
    y_odom = [y_odom, y_odom(end) + u(1) * sin(theta_odom(end))];
    theta_odom = [theta_odom, theta_odom(end) + u(2)];
    
    % update history
    sensor_history(t, :) = sensor_state;
    X_start_history(t, :) = X_start';
    x_true = [x_true, X_start(1)]; 
    y_true = [y_true, X_start(2)];
    theta_true = [theta_true, X_start(3)];
    
    % EKF estimation
%     [X, P] = ekf_estimation(X, P, u, Z);
%     % store ekf estimation
%     x_ekf = [x_ekf, X_start(1)];
%     y_ekf = [y_ekf, X_start(2)];
%     theta_ekf = [theta_ekf, X_start(3)];
    
%     store std variance
    sigma_x = [sigma_x, sqrt(P(1, 1))];
    sigma_y = [sigma_y, sqrt(P(2, 2))];
    sigma_theta = [sigma_theta, sqrt(P(3, 3))];
    
    if sum(sensor_state) > 0
        % at least 1 sensor detect line 
        detect_line_timestep = [detect_line_timestep, t];
    end
    
    % store boundary of ekf estimation
    x_upper = [x_upper, X_start(1) + 3 * sqrt(P(1, 1))];
    x_lower = [x_lower, X_start(1) - 3 * sqrt(P(1, 1))];
    y_upper = [y_upper, X_start(2) + 3 * sqrt(P(2, 2))];
    y_lower = [y_lower, X_start(2) - 3 * sqrt(P(2, 2))];
    theta_upper = [theta_upper, X_start(3) + 3 * sqrt(P(3, 3))];
    theta_lower = [theta_lower, X_start(3) - 3 * sqrt(P(3, 3))];
   
end

if display_sensor
    figure(1)
    plot(X_start_history(:, 1), sensor_history(:, 1), 'r*')
    xlabel("x coordinate")
    ylabel("Sensor state")
    for i = 1 : num_edges - 1
        % vertical line
        v_pos = [i * tile_size - 0.5 * edge_thick, 0, edge_thick, 1.5];
        rectangle('Position', v_pos, 'FaceColor', 'k', 'EdgeColor', 'k');
    end
    ylim([0, 1.5]);
    xlim([-0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick])
    yticks([0, 1]);
    hold off
end

% csvwrite('simulated_data.txt',simulated_data,0,0)
% type simulated_data.txt

csvwrite('v_history.txt',v_history,0,0)
type v_history.txt

end
