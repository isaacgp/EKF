function R = rot_z(theta)
%Calculate the 2D rotation matrix around z-axis

R = [cos(theta),    -sin(theta);
     sin(theta),    cos(theta)];

end

