function [delta_q_r, delta_q_l] = encoder(t, traj)
%Simulate encoder
    if traj == "line_x"
        delta_pulse_left = 2 * 360;
        delta_pulse_right = 2 * 360;
    elseif traj == "diag"
        if t < 2    
            delta_pulse_left = 0 ;
            delta_pulse_right = 2 * 360; 
        else
            delta_pulse_left = 2 * 360; 
            delta_pulse_right = 2 * 360;
        end    
    elseif traj == "circle"
        if t < 20
            delta_pulse_left = 2 * 360;
            delta_pulse_right = 2 * 360;
        else
            delta_pulse_left = 4.5 * 360;
            delta_pulse_right = 5 * 360; 
        end
    else
        error("Undefined trajectory. Choose among {line_x, diag, circle}")
    end
    % Assume encoder returns 360 pulse per revolution
    delta_q_l = delta_pulse_left / 360 ;
    delta_q_r = delta_pulse_right / 360 ;
end