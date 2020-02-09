function [tile_x, tile_y] = tile_localize(X, tile_size, num_edges)
%Localize the tile that contains the robot
%Input:
%   X [3x1]: robot pose
%   tile_size (int): distance between 2 edges
%   num_edges (int): total number of edges along either x axis (y has same
%   number of edges)
%Output:
%   tile_x (int): x index of the inner edge of the tile contains the robot
%   tile_y (int): y index of the lower edge of the tile contains the robot


% Look along x-axis
tile_x = 0;  % y-axis
flag_found_x = false;
while ~flag_found_x && tile_x < num_edges - 1
    if tile_x * tile_size < X(1) && (tile_x + 1) * tile_size >= X(1)
        flag_found_x = true;
    else
        tile_x = tile_x + 1;
    end
end

% Look along y-axis
tile_y = 0;  % y-axis
flag_found_y = false;
while ~flag_found_y && tile_y < num_edges - 1
    if tile_y * tile_size < X(2) && (tile_y + 1) * tile_size >= X(2)
        flag_found_y = true;
    else
        tile_y = tile_y + 1;
    end
end

% Check if robot still in the environment
if ~(flag_found_x && flag_found_y)
    error("Robot is out of the environment."); 
end

end

