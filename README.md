# Coppelia-AStar-Controller

Using Coppelia or V-REP to simulate 2-wheel differential Robot control within a Map (created via Matlab and V-REP).

Basically just start the simulation in V-REP program (in this case the `Map.ttt` file) then run the `launch.m` file with MATLAB.

You can change the start and goal position in `launch.m` file.

The Matlab file will calculate the A-Star Algorithm and find the shortest path to the goal, plot it in figure, send the position of the grid to Coppelia so as to spawn Dummies based on that position. Then Robot just follow the Dummies by the move2goal function.
