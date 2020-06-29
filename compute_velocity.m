function [v, omega] = compute_velocity(waypoint, X)
%Moving to a point
% Compute (v, omega) for robot to move from its current position to
% waypoint. The waypoint is defined in the global frame
% Input:
%   waypoints [2x1] 
%   X [3x1] robot state
% Output:
%   v [1x1] linear velocity
%   omega [1x1] angular velocity

% Get robot dimension
global e r_r r_l

% Control parameter
lambda_v = 0.5;
lambda_omega = 0.5;
d = 0.1;
omega_max = 10;  % maximum wheel speed

% Compute position of waypoint in robot frame
% orientation of robot frame w.r.t global frame
R = [cos(X(3)),     -sin(X(3));
     sin(X(3)),     cos(X(3))];  % Rot Z
r_wp = [R', -R' * X(1:2)] * [waypoint; 1];

% Compute (v*, omega*) (ideal twist)
v_star = lambda_v * (r_wp(1) - d);
omega_star = lambda_omega * atan2(r_wp(2), r_wp(1));

% Define the quadratic program for solving the velocity
Q = eye(2);
r = [v_star;
     omega_star];
K = [r_r/2,     r_l/2;
     r_r/e,     -r_l/e];

% cost function 
H = Q' * Q;
f = -Q' * r;

% inequality constraint
A = [inv(K);
     -inv(K)];
b = zeros(4, 1) + omega_max;

% equality constraint
Aeq = [omega_star,  -v_star];
beq = 0;

% Solve QP
twist = quadprog(H,f,A,b,Aeq,beq);
v = twist(1);
omega = twist(2);

end

