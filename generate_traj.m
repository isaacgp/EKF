function u = generate_traj(t, traj_type)
%Calculate odometry according to trajectory type
% Output:
%   u[2x1] [\Delta D, \Delta theta]

global r_r r_l e

% Wheel displacement
[delta_q_r, delta_q_l] = encoder(t, traj_type);

% Calculate odometry
K = [r_r/2,     r_l/2;
     r_r/e,     -r_l/e];

u = K * [delta_q_r, delta_q_l]';

end

