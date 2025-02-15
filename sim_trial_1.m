clc; clear; close all;

%   first simulation trial where there is a leader robot and the rest follows
%   simple 2 implementation

% Simulation parameters
num_robots = 20;   % Number of swarm robots
dt = 0.1;         % Time step
steps = 500;      % Simulation steps
comm_range = 10;  % Communication range

% Initialize robot positions and velocities
positions = 100 * rand(num_robots, 2); % Random initial positions in a 100x100 area
velocities = randn(num_robots, 2);     % Random initial velocities

% Define leader
leader_index = 1;
leader_position = [50, 50]; % Fixed leader position
leader_velocity = [1, 0];  % Fixed leader velocity

% Flocking parameters
cohesion_weight = 0.01;
alignment_weight = 0.1;
separation_weight = 0.5;
leader_follow_weight = 0.2;
max_speed = 2;

figure;
hold on;
axis([0 100 0 100]);
for t = 1:steps
    new_velocities = velocities;
    for i = 2:num_robots  % Robots follow the leader
        neighbors = vecnorm(positions - positions(i, :), 2, 2) < comm_range;
        neighbors(i) = 0; % Exclude self
        neighbor_indices = find(neighbors);
        
        % Cohesion (move towards center of mass)
        if ~isempty(neighbor_indices)
            center_of_mass = mean(positions(neighbor_indices, :), 1);
            cohesion_force = (center_of_mass - positions(i, :)) * cohesion_weight;
        else
            cohesion_force = [0, 0];
        end
        
        % Alignment (match velocity with neighbors)
        if ~isempty(neighbor_indices)
            avg_velocity = mean(velocities(neighbor_indices, :), 1);
            alignment_force = (avg_velocity - velocities(i, :)) * alignment_weight;
        else
            alignment_force = [0, 0];
        end
        
        % Separation (avoid collisions)
        separation_force = [0, 0];
        for j = neighbor_indices'
            distance = positions(i, :) - positions(j, :);
            if norm(distance) > 0
                separation_force = separation_force + (distance / norm(distance));
            end
        end
        separation_force = separation_force * separation_weight;
        
        % Leader following behavior
        leader_force = (leader_position - positions(i, :)) * leader_follow_weight;
        
        % Update velocity
        new_velocities(i, :) = velocities(i, :) + cohesion_force + alignment_force + separation_force + leader_force;
        
        % Limit speed
        speed = norm(new_velocities(i, :));
        if speed > max_speed
            new_velocities(i, :) = (new_velocities(i, :) / speed) * max_speed;
        end
    end
    
    % Update leader's position
    leader_position = leader_position + leader_velocity * dt;
    positions(leader_index, :) = leader_position;
    velocities(leader_index, :) = leader_velocity;
    
    % Update positions of followers
    positions(2:end, :) = positions(2:end, :) + new_velocities(2:end, :) * dt;
    velocities(2:end, :) = new_velocities(2:end, :);
    
    % Visualization
    cla;
    plot(positions(1, 1), positions(1, 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Leader in red
    plot(positions(2:end, 1), positions(2:end, 2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b'); % Followers in blue
    title(['Time Step: ', num2str(t)]);
    pause(0.01);
end
