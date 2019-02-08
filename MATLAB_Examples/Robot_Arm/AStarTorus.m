function route = AStarTorus (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%      the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%       respectively, the first entry is the row and the second the column.
% Output :
%   route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - red - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 0 0; ...
        1 1 0];

colormap(cmap);


[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;  % Mark free cells
map(input_map)  = 2;  % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

parent = zeros(nrows,ncols);
% meshgrid will `replicate grid vectors' nrows and ncols to produce
% a full grid
% type `help meshgrid' in the Matlab command prompt for more information
[X, Y] = meshgrid (1:ncols, 1:nrows);

xd = dest_coords(1);
yd = dest_coords(2);

H = abs(X-xd)+abs(Y-yd);
H = H';

f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    image(1.5, 1.5, map);
    grid on;
    axis image;
    drawnow;
    
    % Find the node with the minimum distance
    [min_f, current] = min(f(:));
    
    if ((current == dest_node) || isinf(min_f))
        break;
    end
    
    % Update map
    map(current) = 3;         % mark current node as visited
    f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), current);
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    
    if (i>1 && i<=nrows)
        i_north = i-1;
        j_north = j;
        update (i_north,j_north,g(i,j),current);
    end
    if i==1
        i_north = nrows;
        j_north = j;
        update (i_north,j_north,g(i,j),current);
    end
    if (i>1 && i<nrows)
        i_south = i+1;
        j_south = j;
        update (i_south,j_south,g(i,j),current);
    end
    if i==nrows
        i_south = 1;
        j_south = j;
        update (i_south,j_south,g(i,j),current);
    end
    if (j>1 && j<=ncols)
        j_west = j-1;
        i_west = i;
        update (i_west,j_west,g(i,j),current);
    end
    if j==1
        i_west = i;
        j_west = ncols;
        update (i_west,j_west,g(i,j),current);
    end
    if (j>1 && j<ncols)
        j_east =j+1;
        i_east = i;
        update (i_east,j_east,g(i,j),current);
    end
    if j==ncols
        i_east = i;
        j_east = 1;
        update (i_east,j_east,g(i,j),current);
    end
end

    


if (isinf(f(dest_node)))
    route = [];
else
    
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
end

    function update (i,j,g_c,p)
        if ( (map(i,j) ~= 2) && (map(i,j) ~= 3) && (map(i,j) ~= 5) && (g(i,j) > (g_c+1)) )
          
            g(i,j) = g_c+1;
            f(i,j) = g(i,j)+H(i,j);
            map(i,j) = 4;
            parent(i,j) = p;
           
        end
        
    end
    drawMap(1);
    function drawMap(label)
        if label==true
        for k = 2:length(route) - 1        
            map(route(k)) = 7;
            pause(0.1)
       
        image(1.5, 1.5, map);
        
        grid on;
        axis image;
        end
        end
        end
end

