% Underwater Robotic Swarm Simulation with Formation-Following

% Description: Simulates a swarm of underwater robots following a leader in a V-shaped formation.

clc;
clear;
close all;

%% Parameters
numRobots = 20;              % Number of robots in the swarm
timeSteps = 200;             % Number of simulation time steps
dt = 0.1;                    % Time step size
areaSize = [100, 100, 50];   % 3D environment size [x, y, z]
maxSpeed = 2;                % Maximum speed of robots
neighborDist = 20;           % Distance to consider neighbors
obstaclePos = [50, 50, 25];  % Position of a spherical obstacle
obstacleRadius = 15;         % Radius of the obstacle
formationSpacing = 5;        % Spacing between robots in the formation

%% Initialize Robot Positions and Velocities
positions = rand(numRobots, 3) .* areaSize; % Random initial positions
velocities = rand(numRobots, 3) * maxSpeed; % Random initial velocities

% Designate the first robot as the leader
leaderIndex = 1;
leaderPath = [linspace(10, 90, timeSteps); linspace(10, 90, timeSteps); linspace(10, 40, timeSteps)]'; % Leader's path

% Define V-shaped formation offsets
formationOffsets = zeros(numRobots, 3); % Relative positions to the leader
for i = 2:numRobots
    if mod(i, 2) == 0
        formationOffsets(i, :) = [-(i/2)*formationSpacing, (i/2)*formationSpacing, 0]; % Left side of V
    else
        formationOffsets(i, :) = [((i-1)/2)*formationSpacing, ((i-1)/2)*formationSpacing, 0]; % Right side of V
    end
end

%% Visualization Setup
figure;
hold on;
axis([0 areaSize(1) 0 areaSize(2) 0 areaSize(3)]);
view(3); % 3D view
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Underwater Robotic Swarm Simulation with Formation-Following');

% Plot obstacle
[x, y, z] = sphere;
obstacle = surf(x * obstacleRadius + obstaclePos(1), ...
                y * obstacleRadius + obstaclePos(2), ...
                z * obstacleRadius + obstaclePos(3));
set(obstacle, 'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Initialize robot plot
robotPlot = scatter3(positions(:,1), positions(:,2), positions(:,3), 'filled');
leaderPlot = scatter3(positions(leaderIndex,1), positions(leaderIndex,2), positions(leaderIndex,3), 'r', 'filled');

%% Simulation Loop
for t = 1:timeSteps
    % Update leader position
    positions(leaderIndex, :) = leaderPath(t, :);
    
    % Update velocities based on flocking rules and formation
    for i = 1:numRobots
        if i == leaderIndex
            continue; % Leader does not follow flocking rules
        end
        
        separation = [0, 0, 0];
        alignment = [0, 0, 0];
        cohesion = [0, 0, 0];
        followFormation = [0, 0, 0];
        neighborCount = 0;
        
        % Flocking behavior with neighbors
        for j = 1:numRobots
            if i ~= j
                dist = norm(positions(i,:) - positions(j,:));
                if dist < neighborDist
                    % Separation: steer away from neighbors
                    separation = separation - (positions(j,:) - positions(i,:)) / dist;
                    
                    % Alignment: steer towards the average heading of neighbors
                    alignment = alignment + velocities(j,:);
                    
                    % Cohesion: steer towards the center of mass of neighbors
                    cohesion = cohesion + positions(j,:);
                    
                    neighborCount = neighborCount + 1;
                end
            end
        end
        
        % Follow formation: steer towards the desired formation position
        desiredPosition = positions(leaderIndex, :) + formationOffsets(i, :);
        followFormation = (desiredPosition - positions(i,:)) / norm(desiredPosition - positions(i,:));
        
        if neighborCount > 0
            % Normalize alignment and cohesion
            alignment = alignment / neighborCount;
            cohesion = (cohesion / neighborCount - positions(i,:)) / 100;
        end
        
        % Update velocity
        velocities(i,:) = velocities(i,:) + separation * 0.1 + alignment * 0.05 + cohesion * 0.01 + followFormation * 0.2;
        
        % Limit speed
        speed = norm(velocities(i,:));
        if speed > maxSpeed
            velocities(i,:) = velocities(i,:) / speed * maxSpeed;
        end
    end
    
    % Update positions
    positions = positions + velocities * dt;
    
    % Avoid obstacle (simple repulsion)
    for i = 1:numRobots
        distToObstacle = norm(positions(i,:) - obstaclePos);
        if distToObstacle < obstacleRadius + 5
            repulsion = (positions(i,:) - obstaclePos) / distToObstacle^2;
            velocities(i,:) = velocities(i,:) + repulsion * 10;
        end
    end
    
    % Boundary conditions (keep robots within the area)
    positions = max(positions, 0);
    positions = min(positions, areaSize);
    
    % Update visualization
    set(robotPlot, 'XData', positions(:,1), 'YData', positions(:,2), 'ZData', positions(:,3));
    set(leaderPlot, 'XData', positions(leaderIndex,1), 'YData', positions(leaderIndex,2), 'ZData', positions(leaderIndex,3));
    drawnow;
    
    % Pause for animation
    pause(0.05);
end