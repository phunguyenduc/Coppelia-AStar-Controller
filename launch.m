function main()
    % Clear the workspace and command window
    clear;
    clc;

    % Initialize the remote API (make sure to have CoppeliaSim running)
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1); % Close any previously opened connections
    clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

    if clientID > -1
        disp('Connected to CoppeliaSim');

        % Map setting
        map_size = [50, 50];

        % Obstacles definition
        obs0 = [1:map_size(1), ...
                map_size(1):map_size(1):map_size(1) * map_size(2), ...
                1:map_size(1):(map_size(1) - 1) * map_size(2), ...
                map_size(1) * (map_size(2) - 1):map_size(1) * map_size(2)];

        obs1 = [370:1:380,...
                420:1:430,...
                470:1:480,...
                520:1:530,...
                570:1:580,...
                620:1:630,...
                670:1:680,...
                720:1:730,...
                770:1:780,...
                820:1:830,...
                870:1:880,...
                920:1:930];

        obs2 = [1215:1:1220,...
            1265:1:1270,...
            1315:1:1320,...
            1365:1:1370,...
            1415:1:1420,...
            1465:1:1470];

        obs3 = [1435:1:1440,...
            1485:1:1490,...
            1535:1:1540,...
            1585:1:1590,...
            1635:1:1640,...
            1685:1:1690,...
            1735:1:1740,...
            1785:1:1790,...
            1835:1:1840,...
            1885:1:1890,...
            1935:1:1940,...
            1985:1:1990,];

        obstraitim = [2010, 2059, 2110, 2056, 2107, 2007, 2158, 2159, 1958, 1959];

        obstacle = [obs0, obs1, obs2, obs3, obstraitim];


        % Create grid
        grid_map = generate_grid(map_size, obstacle);

        % Define start and goal points
        start_point = [3, 4];
        goal_point = [40, 44];

        % Find path using A* algorithm
        path = A_star(grid_map, start_point, goal_point);

        % Plot grid map and path for visualization
        plot_grid(grid_map);
        hold on;
        plot_path(path, start_point, goal_point);

        % Send path to CoppeliaSim for visualization and get dummy positions
        if ~isempty(path)
            dummy_positions = sendPathToCoppeliaSim(vrep, clientID, path);
            moveRobotAlongPath(vrep, clientID, dummy_positions);
        else
            disp('No path found!');
        end

        % Close the connection to CoppeliaSim
        vrep.simxFinish(clientID);
    else
        disp('Failed to connect to CoppeliaSim');
    end

    % Delete the remote API object
    vrep.delete();
end

function grid_map = generate_grid(size, obstacle)
    grid_map = ones(size(1), size(2));
    grid_map(obstacle) = 2;
end

function plot_grid(grid_map)
    cmap = [1 1 1; ...
            0 0 0];
    colormap(cmap);

    [rows, cols] = size(grid_map);
    image(1.5, 1.5, grid_map);
    grid on;
    set(gca,'xtick', 1:cols, 'ytick', 1:rows);
    axis image;

    % Set y-axis direction to normal
    set(gca, 'YDir', 'normal');

    for row = 1:rows
        line([1, cols + 1], [row, row]);
    end
    for col = 1:cols
        line([col, col], [1, rows + 1]);
    end
end

function plot_path(path, start_point, goal_point)
    plot(start_point(2), start_point(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(goal_point(2), goal_point(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    for i = 1:size(path, 1)
        plot(path(i, 2), path(i, 1), 'g.', 'MarkerSize', 20);
    end
end

function path = A_star(grid_map, start, goal)
    % Define the heuristic function (Euclidean distance)
    heuristic = @(a, b) norm(a - b);

    % Initialize open and closed lists
    open_list = [];
    closed_list = [];

    % Initialize start node
    start_node.g = 0;
    start_node.h = heuristic(start, goal);
    start_node.f = start_node.g + start_node.h;
    start_node.pos = start;
    start_node.parent = [];

    % Add start node to open list
    open_list = [open_list, start_node];

    while ~isempty(open_list)
        % Find node with lowest f value in open list
        [~, current_index] = min([open_list.f]);
        current_node = open_list(current_index);

        % Check if goal is reached
        if isequal(current_node.pos, goal)
            % Reconstruct path
            path = [];
            while ~isempty(current_node.parent)
                path = [current_node.pos; path];
                current_node = current_node.parent;
            end
            path = [current_node.pos; path];
            return;
        end
        
        diagona1_cost = 1;
        
        % Move current node from open to closed list
        open_list(current_index) = [];
        closed_list = [closed_list, current_node];

        % Generate successor nodes
        successors = [];
        % Define movement costs
        straight_cost = 1;
        diagonal_cost = sqrt(2);

        % Loop through surrounding directions (8 directions: left, right, up, down, and 4 diagonals)
        for dx = -1:1
            for dy = -1:1
                % Skip the case of not moving (dx = 0 and dy = 0)
                if dx == 0 && dy == 0
                    continue;
                end

                % Calculate the position of the successor node
                neighbor_pos = current_node.pos + [dx, dy];

                % Check if the position of the successor node is within the grid
                if neighbor_pos(1) >= 1 && neighbor_pos(1) <= size(grid_map, 1) && ...
                   neighbor_pos(2) >= 1 && neighbor_pos(2) <= size(grid_map, 2) && ...
                   grid_map(neighbor_pos(1), neighbor_pos(2)) ~= 2
                    % Determine the movement cost
                    if abs(dx) + abs(dy) == 2
                        cost = diagona1_cost; % Cost for diagonal movement
                    else
                        cost = straight_cost; % Cost for straight movement
                    end

                    successor.g = current_node.g + cost; % Compute the cost to move to the successor node
                    successor.h = heuristic(neighbor_pos, goal); % Compute heuristic (estimated distance to goal)
                    successor.f = successor.g + successor.h; % Total cost f = g + h
                    successor.pos = neighbor_pos; % Position of the successor node
                    successor.parent = current_node; % Set the current node as the parent

                    % Check if the successor is already in the closed list with a lower f value
                    if any(isequal(successor.pos, [closed_list.pos])) && ...
                       successor.f >= closed_list(find(isequal(successor.pos, [closed_list.pos]))).f
                        continue;
                    end
                    
                    super_cost = diagonal_cost;
                    % Check if the successor is already in the open list with a lower f value
                    open_index = find(isequal(successor.pos, [open_list.pos]));
                    if ~isempty(open_index) && successor.f >= open_list(open_index).f
                        continue;
                    end

                    % Add the successor to the open list
                    open_list = [open_list, successor];
                end
            end
        end
    end

    % No path found
    disp('Không tìm thấy đường đi!');
    path = [];
end

function dummy_positions = sendPathToCoppeliaSim(vrep, clientID, path)
    dummy_positions = [];
    for i = 1:size(path, 1)
        pos = path(i, :);
        % Convert grid coordinates to CoppeliaSim world coordinates (customize as needed)
        x = (pos(2) - 1) * 0.2; % Scale to match C
        y = (pos(1) - 1) * 0.2; % Scale to match CoppeliaSim coordinates
        z = 0; % Assuming the path is on a flat surface

        % Create a dummy object at the path position
        % Color array needs to have 12 values
        colorArray = [0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0];
        [res, dummyHandle] = vrep.simxCreateDummy(clientID, 0.1, colorArray, vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok
            vrep.simxSetObjectPosition(clientID, dummyHandle, -1, [x, y, z], vrep.simx_opmode_blocking);
            dummy_positions = [dummy_positions; [x, y]]; % Store dummy position
        end
    end
end
function moveRobotAlongPath(vrep, clientID, dummy_positions)
    [~, left_motor] = vrep.simxGetObjectHandle(clientID, 'motor_left', vrep.simx_opmode_blocking);
    [~, right_motor] = vrep.simxGetObjectHandle(clientID, 'motor_right', vrep.simx_opmode_blocking);
    [~, robotHandle1] = vrep.simxGetObjectHandle(clientID, 'GPS', vrep.simx_opmode_blocking);
    [~, robotHandle2] = vrep.simxGetObjectHandle(clientID, 'GyroSensor', vrep.simx_opmode_blocking);

    % Streaming positions of the robot
    [~, Posirobot] = vrep.simxGetObjectPosition(clientID, robotHandle1, -1, vrep.simx_opmode_streaming);
    [~, Orierobot] = vrep.simxGetObjectOrientation(clientID, robotHandle2, -1, vrep.simx_opmode_streaming);

    % Move the robot along the path
    for i = 1:size(dummy_positions, 1)
        goal = [dummy_positions(i, 1), dummy_positions(i, 2)]; % Get the position of the current dummy
        isReached = false;
        while ~isReached
            isReached = move2goal(clientID, vrep, left_motor, right_motor, robotHandle1, robotHandle2, goal);
        end
    end
end
function [isReached] = move2goal(clientID,vrep,left_motor,right_motor,robotHandle1,robotHandle2,goal)
%% initialize parameters
% move to goal behavior's parameters
d_m2g = 0.1; % offset den dich
a_m2g = 70;

% K for PID controller
Kp = 300;
Ki = 0.01;
Kd = 50;
dt = 0.05;

%% Initialize PID variables
persistent prevError integError
if isempty(prevError)
    prevError = 0;
    integError = 0;
end

% Feature of robot
R = 0.03; % wheel's radius (m)
L = 0.1665; % the length between two wheels (m)
MAX_SPEED = 200;

%% Read proximity sensor, get robot's heading and robot's position
[~,robotPosition] = vrep.simxGetObjectPosition(clientID,robotHandle1,-1,vrep.simx_opmode_buffer);
robotOxy = [robotPosition(1),robotPosition(2)];

[~,robotHeading] = vrep.simxGetObjectOrientation(clientID,robotHandle2,-1,vrep.simx_opmode_buffer);
% Compute V_m2g
V_m2g = goal - robotOxy;
dist2goal = sqrt(V_m2g(1)^2+V_m2g(2)^2);

if norm(V_m2g) ~= 0
    V_m2g = a_m2g*(V_m2g/norm(V_m2g));
end

%% Set value if isObstacleAhead and issReached
if dist2goal < d_m2g
    isReached = true;
else
    isReached = false;
end

%% Control robot using vector V_m2g
% Get desired heading angle
desiredOrientation = atan2(V_m2g(2),V_m2g(1));
errorAngle = desiredOrientation - robotHeading(3);

if(abs(errorAngle) > pi)
    if(errorAngle < 0)
        errorAngle = errorAngle + 2*pi;
    else
        errorAngle = errorAngle - 2*pi;
    end
end

% PID cal
integError = integError + errorAngle*dt;
derivError = (errorAngle - prevError)/dt;
prevError = errorAngle;

% Compute rotational velocity of robot
%omega = Kp*errorAngle;
omega = Kp*errorAngle + Ki*integError + Kd*derivError;

% Compute linear velocity of robot
v = norm(V_m2g);

% Compute vr and vl to control right wheel and left wheel of the robot
vr = (2*v + omega*L)/2*R;
vl = (2*v - omega*L)/2*R;

if vr > MAX_SPEED
    vr = MAX_SPEED;
elseif vr < -MAX_SPEED
    vr = -MAX_SPEED;
end

if vl > MAX_SPEED
    vl = MAX_SPEED;
elseif vl < -MAX_SPEED
    vl = -MAX_SPEED;
end

% Set target velocity (stop if the robot reached goal)
if isReached == false
    vrep.simxSetJointTargetVelocity(clientID,left_motor,vl,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,right_motor,vr,vrep.simx_opmode_oneshot);
else
    vrep.simxSetJointTargetVelocity(clientID,left_motor,0,vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,right_motor,0,vrep.simx_opmode_oneshot);
    pause(0.2);
end
 disp(['Current target position: ', num2str(goal)]);
end