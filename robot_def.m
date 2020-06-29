function [e, r_r, r_l, sensor_pos, encoder_res] = robot_def()

    % Dimensions of Robot
    r_r = 0.07;  % right wheel radius.
    r_l = 0.07;  % left wheel radius.
    e = 0.2;  % track gauge (i.e. distance between two wheels)

    % Define sensors in robot body frame
    n_s = 2;  % number of sensors
    rho = []; % distance from origin of robot frame to sensor
    lambda = []; % angle between the x-axis of robot frame and sensor
    sensor_pos = []; % each column is polar coordinate of a sensor

    rho(1) = 0.08;
    rho(2) = 0.08;
    lambda(1) = pi/2;
    lambda(2) = -pi/2;
    
    for i = 1 : n_s
        s = [rho(i);
             lambda(i)];  % polar coordinate of a sensor in robot local frame
        sensor_pos = [sensor_pos, s];
    end
    
    encoder_res = 360;  % 360 pulse per round

end

