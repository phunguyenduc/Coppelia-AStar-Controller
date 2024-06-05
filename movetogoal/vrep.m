vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if(clientID > -1)
    disp('Connected')
    [~,left_motor] = vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [~,right_motor] = vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);

    [~,robotHandle1] = vrep.simxGetObjectHandle(clientID,'GPS',vrep.simx_opmode_blocking);
    [~,robotHandle2] = vrep.simxGetObjectHandle(clientID,'GyroSensor',vrep.simx_opmode_blocking);

    [~,Posirobot] = vrep.simxGetObjectPosition(clientID,robotHandle1,-1,vrep.simx_opmode_streaming);
    [~,Orierobot] = vrep.simxGetObjectOrientation(clientID,robotHandle2,-1,vrep.simx_opmode_streaming);
    while true
        goal = [0, 0];
        a = move2goal(clientID,vrep,left_motor,right_motor,robotHandle1,robotHandle2,goal);
    end
else
    disp('Failed connecting')
end

function [isReached] = move2goal(clientID,vrep,left_motor,right_motor,robotHandle1,robotHandle2,goal)
%% initialize parameters
% move to goal behavior's parameters
d_m2g = 0.05; % offset den dich
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
MAX_SPEED = 5;

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
end

end