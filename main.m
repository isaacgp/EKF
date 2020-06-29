%EKF Localization main file
clear 
close all
rng('default') % For reproducibility
clc

display_sensor = true;  % change to true to display sensor state

global tile_size num_edges edge_thick Q_gamma d_Maha_threshold Q_wheel e r_r r_l delta_t encoder_res


filename = 'simulated_data.txt';
last_t = size(csvread(filename),1); % final time step
fid = fopen('simulated_data.txt');
simulated_data = textscan(fgetl(fid),'%f','delimiter',',') ;
simulated_data = simulated_data{1,1}';
filename2 = 'v_history.txt';
v_history = csvread(filename2);

wp = [simulated_data];
simulated_data = textscan(fgetl(fid),'%f','delimiter',',') ;
simulated_data = simulated_data{1,1}';
wp = [wp;simulated_data];
simulated_data = textscan(fgetl(fid),'%f','delimiter',',') ;
simulated_data = simulated_data{1,1}';

last_wp_idx = 1;
                                  
% Initialize true robot pose
X_start = [0.1, 2.2, 0]'; 

% Get initial estimation of robot state & necessary covariance matrices
[X, P, Q_wheel, Q_gamma, d_Maha_threshold] = covariance_def(X_start);

maha_history = [];
re_maha_history = [];
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

switch size(simulated_data,2)
    case 7
        X_start = [simulated_data(1:3)'];
        sensor_state = [simulated_data(4:5)'];
        u = [simulated_data(6:7)];
        Z = [];
    case 12
        X_start = [simulated_data(1:3)'];
        sensor_state = [simulated_data(4:5)'];
        u = [simulated_data(6:7)];
        Z = [simulated_data(8:end)]; 
    case 17
        X_start = [simulated_data(1:3)'];
        sensor_state = [simulated_data(4:5)'];
        u = [simulated_data(6:7)];
        Z = [simulated_data(8:12);
                simulated_data(13:17)];      
    case 22
        X_start = [simulated_data(1:3)'];
        sensor_state = [simulated_data(4:5)'];
        u = [simulated_data(6:7)];
        Z = [simulated_data(8:12);
                simulated_data(13:17);
                simulated_data(18:22)];           
end
    simulated_data = textscan(fgetl(fid),'%f','delimiter',',') ;
    simulated_data = simulated_data{1,1}';
    
for t=4:last_t

    switch size(simulated_data,2)
        case 7
            X_start = [X_start,simulated_data(1:3)'];
            sensor_state = [sensor_state, simulated_data(4:5)'];
            u = [ u; simulated_data(6:7)];
        case 12
            X_start = [X_start,simulated_data(1:3)'];
            sensor_state = [sensor_state, simulated_data(4:5)'];
            u = [ u; simulated_data(6:7)];
            Z = [Z; simulated_data(8:end)]; 
        case 17
            X_start = [X_start,simulated_data(1:3)'];
            sensor_state = [sensor_state, simulated_data(4:5)'];
            u = [ u; simulated_data(6:7)];
            Z = [Z; simulated_data(8:12);
                    simulated_data(13:17)];      
        case 22
            X_start = [X_start,simulated_data(1:3)'];
            sensor_state = [sensor_state, simulated_data(4:5)'];
            u = [ u; simulated_data(6:7)];
            Z = [Z; simulated_data(8:12);
                    simulated_data(13:17);
                    simulated_data(18:22)];          
    end
    if t <last_t
        simulated_data = textscan(fgetl(fid),'%f','delimiter',',') ;
        simulated_data = simulated_data{1,1}';
    end
end
fclose(fid);
%     if( size(simulated_data(t,:),2) == 7 )
        
%     elseif
%     if ( size(simulated_data(t+1,:),2) == 5 )
%         Z = 
%     end
for t = 1 : last_t-2
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
    sensor_history(t, :) = sensor_state(:,t);
    X_start_history(t, :) = X_start(:,t)';
    x_true = [x_true, X_start(1,t)]; 
    y_true = [y_true, X_start(2,t)];
    theta_true = [theta_true, X_start(3,t)];
    
    % EKF estimation
    j=0;
    my_Z = [];
    for i=1:size(Z,1)
        if (Z(i,end) == t && j==0)
            my_Z = Z(i,:); 
            j=1;
        elseif (Z(i,end) == t && j==1)
             my_Z = [ my_Z; Z(i,:) ];
        end
    end
    
    %% June 23rd edition
    [X, P, P_bar, maha_dis, re_maha_dis] = ekf_estimation(X, P, u(t,:), my_Z');
    if sensor_state(1, t) > 0 || sensor_state(2, t) > 0
        disp('Have measurement, difference between P_predict & P_update')
        diag(P) - diag(P_bar)
    end
    %%
    % store ekf estimation
    maha_history = [maha_history;maha_dis];
    re_maha_history = [re_maha_history;re_maha_dis];
    x_ekf = [x_ekf, X(1)];
    y_ekf = [y_ekf, X(2)];
    theta_ekf = [theta_ekf, X(3)];
    
    % store std variance
    sigma_x = [sigma_x, sqrt(P(1, 1))];
    sigma_y = [sigma_y, sqrt(P(2, 2))];
    sigma_theta = [sigma_theta, sqrt(P(3, 3))];
    
    if sum(sensor_state(:,t)) > 0
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
    
end
disp("Simulation has finished.")



%% Plots:
plot_state(X, P, X_start(:,t), wp, x_true, y_true, x_odom, y_odom, theta_odom, x_ekf, y_ekf)
% plot_state_sensors(X, P, X_start(:,t), wp, x_true, y_true, x_odom, y_odom, theta_odom, x_ekf, y_ekf, Z)
  
% Display 1st sensor state
if display_sensor
    figure(2)
    subplot(221)
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
    title("Sensor 1 (Pi/2) State, detecting any line")
hold on
subplot(222)
    plot(X_start_history(:, 2), sensor_history(:, 1), 'r*')
    xlabel("y coordinate")
    ylabel("Sensor state")
    for i = 1 : num_edges - 1
        % horizontal line
        v_pos = [i * tile_size - 0.5 * edge_thick, 0, edge_thick, 1.5];
        rectangle('Position', v_pos, 'FaceColor', 'k', 'EdgeColor', 'k');
    end
    ylim([0, 1.5]);
    xlim([-0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick])
    yticks([0, 1]);
%     title("Sensor 2 (-Pi/2) State, detecting any line")
    title("Sensor 1 (Pi/2) State, detecting any line")
%     hold off
    subplot(223)
    plot(X_start_history(:, 1), sensor_history(:, 2), 'r*')
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
    title("Sensor 2 (-Pi/2) State, detecting any line")
hold on
subplot(224)
    plot(X_start_history(:, 2), sensor_history(:, 2), 'r*')
    xlabel("y coordinate")
    ylabel("Sensor state")
    for i = 1 : num_edges - 1
        % horizontal line
        v_pos = [i * tile_size - 0.5 * edge_thick, 0, edge_thick, 1.5];
        rectangle('Position', v_pos, 'FaceColor', 'k', 'EdgeColor', 'k');
    end
    ylim([0, 1.5]);
    xlim([-0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick])
    yticks([0, 1]);
%     title("Sensor 2 (-Pi/2) State, detecting any line")
    title("Sensor 2 (-Pi/2) State, detecting any line")
%     hold off
end

figure(4)
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

figure(5)
hold on
xlabel("timestep")
ylabel("meters")
title('Sigma - Standard deviation')
p_sigma_x = plot(sigma_x,'b-');
p_sigma_y = plot(sigma_y,'g-');
p_sigma_theta = plot(sigma_theta,'r-');
ylim([0, max(max(max(sigma_theta), max(sigma_x)),max(sigma_y))])
xlim([0,last_t])
line_k = 0; line_c = 0; line_m = 0; %declaring variables
for i = 1 : last_t-2 %plot lines
    if sensor_state(1,i) > 0
        line_c = plot( [i-1, i-1], [0, max(max(max(sigma_theta), max(sigma_x)),max(sigma_y))], 'c' );
    end
    if sensor_state(2,i) > 0
        line_m = plot( [i-1, i-1], [0, max(max(max(sigma_theta), max(sigma_x)),max(sigma_y))], 'm' );
    end
    if sum(sensor_state (:,i)) == 2
        line_k = plot( [i-1, i-1], [0, max(max(max(sigma_theta), max(sigma_x)),max(sigma_y))], 'k' ); 
    end
end
if (line_k == 0 && line_c == 0 && line_m == 0) % legends
    legend([p_sigma_x, p_sigma_y, p_sigma_theta], ["Sigma x", "Sigma y", "Sigma Theta"])
elseif (line_c == 0 )
    legend([p_sigma_x, p_sigma_y, p_sigma_theta, line_m], ["Sigma x", "Sigma y", "Sigma Theta", "Sensor 2"])
elseif (line_m == 0 )
    legend([p_sigma_x, p_sigma_y, p_sigma_theta, line_c], ["Sigma x", "Sigma y", "Sigma Theta", "Sensor 1"])
elseif (line_k == 0 )
    legend([p_sigma_x, p_sigma_y, p_sigma_theta, line_c, line_m], ["Sigma x", "Sigma y", "Sigma Theta", "Sensor 1", "Sensor 2"])
else 
    legend([p_sigma_x, p_sigma_y, p_sigma_theta, line_k, line_c, line_m], ["Sigma x", "Sigma y", "Sigma Theta", "Both sensors", "Sensor 1", "Sensor 2"])
end

figure(6)
hold on
v_history_linear = v_history(:, 1);
v_history_angular = v_history(:, 2);
p1_v_history = plot(v_history_linear(1:end),'b-');
p2_v_history = plot(v_history_angular(1:end),'r-');
legend([ p1_v_history, p2_v_history], ["Linear Velocity", "Angular Velocity"])
xlabel("timestep")
ylabel("m/s and rad/s")
title("Velocity")

figure(7)
hold on
if isempty(maha_history)
    maha_history_timestep = [];
    maha_history_value = [];
else
    maha_history_timestep = maha_history(:, 2);
    maha_history_value = maha_history(:, 1);
end
if isempty(re_maha_history)
    re_maha_history_timestep =[];
    re_maha_history_value =[];
else
    re_maha_history_timestep = re_maha_history(:, 2);
    re_maha_history_value = re_maha_history(:, 1);
end
p1_m_history = plot(maha_history_timestep,maha_history_value ,'og');
p2_m_history = plot(re_maha_history_timestep,re_maha_history_value ,'or');
p3_line = line([0,65],[d_Maha_threshold, d_Maha_threshold]);
legend([ p3_line, p1_m_history, p2_m_history ], ["Mahalabonis Threshold Square", "Valid Measurements", "Rejected Measurements"])
xlabel("timestep")
ylabel("value")
title("Mahalanobis")

odom = [x_odom;
        y_odom;
        theta_odom];
ekf = [x_ekf;
       y_ekf;
       theta_odom];  
   my_norm_odom=[];
   my_norm_ekf=[];
   for i=1:64
   my_norm_odom = [my_norm_odom , norm(X_start(:,i)-odom(:,i))];
   my_norm_ekf = [my_norm_ekf , norm(X_start(:,i)-ekf(:,i))];
   end
figure(8)
hold on
plot(my_norm_odom,'b');
plot(my_norm_ekf,'r');
legend("True-Odometry", "True-EKF")
ylabel("no unit")
xlabel("timestep")
title("Error")


