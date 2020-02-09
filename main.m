%EKF Localization main file
clear 
close all
rng('default') % For reproducibility
clc

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
[X, P, Q_wheel, Q_gamma, d_Maha_threshold] = covariance_def(X_start);

last_t = 65;  % final time step
sensor_history = zeros(last_t, size(sensor_pos, 2));
X_start_history = zeros(last_t, 3);
v_history = zeros(last_t,1);
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
x_ekf = [X_start(1)];
y_ekf = [X_start(2)];
theta_ekf = [X_start(3)];
sigma_x = [];
sigma_y = [];
sigma_theta = [];
detect_line_timestep = [];

%[simulated_data, v_history] = main_trajectory();
filename = 'simulated_data.txt';
last_t = size(csvread(filename),1);
fid = fopen('simulated_data.txt');
simulated_data = fgetl(fid);

filename2 = 'v_history.txt';
v_history = csvread(filename2);
% 
%         X_start = simulated_data(:,1:3);
%         Z = [simulated_data(:,4:6);simulated_data(:,7:9);simulated_data(:,10:12) ];
%         sensor_state = simulated_data(:,13:14);
%         u = simulated_data(:,15:16);
for t=1:size(simulated_data,1)
    simulated_data = fgetl(fid);
    switch size(simulated_data(:),2)
        case 7
            X_start = [X_start;simulated_data(1:3)];
            sensor_state = [sensor_state; simulated_data(4)];
            u = [ u; simulated_data(5:7)];
        case 12
            X_start = [X_start;simulated_data(1:3)];
            sensor_state = [sensor_state; simulated_data(4)];
            u = [ u; simulated_data(5:7)];
            Z = [Z; simulated_data(8:end)]; 
        case 17
            X_start = [X_start;simulated_data(1:3)];
            sensor_state = [sensor_state; simulated_data(4)];
            u = [ u; simulated_data(5:7)];
            Z = [Z; simulated_data(8:end);
                    simulated_data(11:end)];      
        case 22
            X_start = [X_start;simulated_data(1:3)];
            sensor_state = [sensor_state; simulated_data(4)];
            u = [ u; simulated_data(5:7)];
            Z = [Z; simulated_data(8:end);
                    simulated_data(11:end);
                    simulated_data(14:end)];          
    end
end
fclose(fid);
%     if( size(simulated_data(t,:),2) == 7 )
        
%     elseif
%     if ( size(simulated_data(t+1,:),2) == 5 )
%         Z = 
%     end
for t = 1 : last_t
    % Choose waypoints
    if norm(X(1:2) - wp(:, last_wp_idx)) < 0.5
        last_wp_idx = last_wp_idx + 1;
    end
    
    if last_wp_idx > length(wp)  % travered all waypoints
        break 
    end
    
    % Compute odometry estimation
    x_odom = [x_odom, x_odom(end) + u(t,1) * cos(theta_odom(end))];
    y_odom = [y_odom, y_odom(end) + u(t,1) * sin(theta_odom(end))];
    theta_odom = [theta_odom, theta_odom(end) + u(t,2)];
    
    % update history
    sensor_history(t, :) = sensor_state(t,:);
    X_start_history(t, :) = X_start(t,:)';
    x_true = [x_true, X_start(t,1)]; 
    y_true = [y_true, X_start(t,2)];
    theta_true = [theta_true, X_start(t,3)];
    
    % EKF estimation
    [X, P] = ekf_estimation(X, P, u(t,:), [Z((t*3)-2,:);Z((t*3)-1,:);Z((t*3),:)]);
    % store ekf estimation
    x_ekf = [x_ekf, X(1)];
    y_ekf = [y_ekf, X(2)];
    theta_ekf = [theta_ekf, X(3)];
    
    % store std variance
    sigma_x = [sigma_x, sqrt(P(1, 1))];
    sigma_y = [sigma_y, sqrt(P(2, 2))];
    sigma_theta = [sigma_theta, sqrt(P(3, 3))];
    
    if sum(sensor_state(t,:)) > 0
        % at least 1 sensor detect line 
        detect_line_timestep = [detect_line_timestep, t];
    end
    
    % store boundary of ekf estimation
    x_upper = [x_upper, X(1) + 3 * sqrt(P(1, 1))];
    x_lower = [x_lower, X(1) - 3 * sqrt(P(1, 1))];
    y_upper = [y_upper, X(2) + 3 * sqrt(P(2, 2))];
    y_lower = [y_lower, X(2) - 3 * sqrt(P(2, 2))];
    theta_upper = [theta_upper, X(3) + 3 * sqrt(P(3, 3))];
    theta_lower = [theta_lower, X(3) - 3 * sqrt(P(3, 3))];
%% Plots:    
    % display state
    plot_state(X, P, X_start(t,:), wp, x_true, y_true, x_odom, y_odom, theta_odom, x_ekf, y_ekf)
    
end

disp("Simulation has finished.")
%% Plots: 
% Display 1st sensor state
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

figure(2)
subplot(311)
hold on
plot(x_true, 'b');
plot(x_upper, 'r');
plot(x_lower, 'r');
legend("true", "upper", "lower")
ylabel("x-coordinate (m)")
title("True value of x-coordinate compared to its estimation")
subplot(312)
hold on
plot(y_true, 'b');
plot(y_upper, 'r');
plot(y_lower, 'r');
legend("true", "upper", "lower")
ylabel("y-coordinate (m)")
title("True value of y-coordinate compared to its estimation")
subplot(313)
hold on
plot(theta_true, 'b');
plot(theta_upper, 'r');
plot(theta_lower, 'r');
legend("true", "upper", "lower")
ylabel("theta (rad)")
title("True value of theta compared to its estimation")

%%
figure(3)
hold on
for i = 1 : length(detect_line_timestep)
    plot([detect_line_timestep(i), detect_line_timestep(i)], [0, 2])
end
xlabel("timestep")
ylabel("meters")
title('Sigma - Standard deviation')
p_sigma_x = plot(sigma_x,'b-');
ylim([0, 1.2 * max(sigma_x)])
p_sigma_y = plot(sigma_y,'g-');
ylim([0, 1.2 * max(sigma_y)])
p_sigma_theta = plot(sigma_theta,'r-');
ylim([0, 1.2 * max(sigma_theta)])

legend([p_sigma_x, p_sigma_y, p_sigma_theta], ["Sigma x", "Sigma y", "Sigma Theta"])


figure(4)
hold on
v_history_linear = v_history(:, 1);
v_history_angular = v_history(:, 2);
p1_v_history = plot(v_history_linear((size(v_history_linear,1)-last_t):end),'b-');
p2_v_history = plot(v_history_angular((size(v_history_angular,1)-last_t):end),'r-');
% ylim([0, 1.2 * max(sigma_x)])
legend([ p1_v_history, p2_v_history], ["Linear Velocity", "Angular Velocity"])
xlabel("timestep")
ylabel("m/s and rad/s")
title("Velocity")

figure(5)
hold on
v_history_linear = v_history(:, 1);
v_history_angular = v_history(:, 2);
p1_v_history = plot(v_history_linear((size(v_history_linear,1)-last_t):end),'b-');
p2_v_history = plot(v_history_angular((size(v_history_angular,1)-last_t):end),'r-');
% ylim([0, 1.2 * max(sigma_x)])
legend([ p1_v_history, p2_v_history], ["Linear Velocity", "Angular Velocity"])
xlabel("timestep")
ylabel("m/s and rad/s")
title("Velocity")
