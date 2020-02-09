function plot_state(mu, sigma, X, waypoints, x_true, y_true, x_odom, y_odom, theta_odom, x_ekf, y_ekf)
    % Visualizes the state of the EKF SLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - current robot pose estimate (red)
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)

    global tile_size num_edges edge_thick
    
    clf;
    hold on
    % Draw environment
    xlim([-0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick])
    ylim([-0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick])
    
    for i = 1 : num_edges - 1
        % vertical line
        v_pos = [i * tile_size - 0.5 * edge_thick, 0, edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick];
        rectangle('Position', v_pos, 'FaceColor', 'k', 'EdgeColor', 'k');
        % horizontal line
        h_pos = [0, i * tile_size - 0.5 * edge_thick, (num_edges - 1) * tile_size + 0.5 * edge_thick, edge_thick];
        rectangle('Position', h_pos, 'FaceColor', 'k', 'EdgeColor', 'k');
    end
    % Draw waypoints
    plot(waypoints(1,:), waypoints(2,:), 'or');
    
    % Draw robot
%     drawprobellipse(mu, sigma, 0.6, 'r');
    drawrobot(mu, 'r', 3, 0.3, 0.3);
    drawrobot(X, 'b', 3, 0.3, 0.3);
    drawrobot([x_odom(end), y_odom(end), theta_odom(end)]', 'g', 3, 0.3, 0.3);
    
    
    % plot 3 trajectories (true, odom, ekf)
    true_traj = plot(x_true, y_true, 'b-');
    odom_traj = plot(x_odom, y_odom, 'g-');
    ekf_traj = plot(x_ekf, y_ekf, 'r-');
    legend([true_traj, odom_traj, ekf_traj], ["True", "Odom", "EKF"])
    xlabel('Distance along X in the room (meters)')
    ylabel('Distance along Y in the room (meters)') 
    title('(2,0) Differential Robot')
    
    hold off

    drawnow;
    pause(0.05);
end
