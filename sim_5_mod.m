clc; clear; close all;

%% Simulation Parameters
num_trials = 10;       % Number of trials
num_robots_values = [5, 10, 15, 20]; % Different robot counts to test
arena_size = [100, 50];
wall_x = arena_size(1)/2;
hole_y_range = [20, 30];
dt = 0.1;
steps = 1000;
speed = 2.0;

%% Visualization - Single Simulation
num_robots = 10;
robots(num_robots) = struct('pos', [], 'vel', [], 'id', []);
circle_center = [wall_x + 20, mean(hole_y_range)];
circle_radius = 10;

for i = 1:num_robots
    robots(i).pos = [rand()*wall_x/2, rand()*arena_size(2)];
    robots(i).vel = [speed, 0];
    robots(i).id = i;
end

figure;
for t = 1:steps
    clf; hold on;
    axis([0, arena_size(1), 0, arena_size(2)]);
    
    rectangle('Position', [wall_x-1, 0, 2, hole_y_range(1)], 'FaceColor', 'k');
    rectangle('Position', [wall_x-1, hole_y_range(2), 2, arena_size(2)-hole_y_range(2)], 'FaceColor', 'k');
    
    for i = 1:num_robots
        if robots(i).pos(1) < wall_x - 2
            if robots(i).pos(2) < hole_y_range(1) || robots(i).pos(2) > hole_y_range(2)
                robots(i).vel = [0, sign(hole_y_range(1) + hole_y_range(2) - 2 * robots(i).pos(2)) * speed];
            else
                robots(i).vel = [speed, 0];
            end
        elseif robots(i).pos(1) > wall_x
            theta = 2*pi * (i-1) / num_robots;
            robots(i).pos = robots(i).pos + 0.05 * ([circle_center + circle_radius * [cos(theta), sin(theta)]] - robots(i).pos);
            robots(i).vel = [0, 0];
        end
        
        robots(i).pos = robots(i).pos + robots(i).vel * dt;
        plot(robots(i).pos(1), robots(i).pos(2), 'bo', 'MarkerFaceColor', 'b');
    end
    
    pause(0.05);
end

%% Statistical Analysis - Multiple Trials
results = zeros(length(num_robots_values), num_trials);

for r = 1:length(num_robots_values)
    num_robots = num_robots_values(r);
    for trial = 1:num_trials
        robots(num_robots) = struct('pos', [], 'vel', [], 'id', []);

        for i = 1:num_robots
            robots(i).pos = [rand()*wall_x/2, rand()*arena_size(2)];
            robots(i).vel = [speed, 0];
            robots(i).id = i;
        end

        time_steps = 0;
        while time_steps < steps
            time_steps = time_steps + 1;
            all_stopped = true;
            
            for i = 1:num_robots
                if robots(i).pos(1) < wall_x - 2
                    if robots(i).pos(2) < hole_y_range(1) || robots(i).pos(2) > hole_y_range(2)
                        robots(i).vel = [0, sign(hole_y_range(1) + hole_y_range(2) - 2 * robots(i).pos(2)) * speed];
                    else
                        robots(i).vel = [speed, 0];
                    end
                elseif robots(i).pos(1) > wall_x
                    theta = 2*pi * (i-1) / num_robots;
                    robots(i).pos = robots(i).pos + 0.05 * ([circle_center + circle_radius * [cos(theta), sin(theta)]] - robots(i).pos);
                    robots(i).vel = [0, 0];
                end
                
                robots(i).pos = robots(i).pos + robots(i).vel * dt;
            end
            
            if all_stopped
                all_stopped = all([robots(:).vel] == 0);
                if all_stopped
                    break;
                end
            end
        end
        results(r, trial) = time_steps;
    end
end

%% --- Existing code up through your first errorbar plot ---
% (clc; clear; close all; … your Simulation + first Statistical Analysis …)

%% Varying Wall‐Opening Size (with 10 robots fixed)
gap_sizes    = [5, 10, 15, 20];    % different opening heights to test
num_trials   = 10;                 % same number of trials
num_robots   = 10;                 % fixed robot count
center_y     = mean(hole_y_range); % use your original hole center (≈25)
results_gap  = zeros(length(gap_sizes), num_trials);

for g = 1:length(gap_sizes)
    gap = gap_sizes(g);
    % recompute the opening bounds around the same center
    hole_y = [ center_y - gap/2,  center_y + gap/2 ];
    
    for trial = 1:num_trials
        % --- initialize robots exactly as before ---
        robots(num_robots) = struct('pos',[],'vel',[],'id',[]);
        for i = 1:num_robots
            robots(i).pos = [ rand()*wall_x/2,  rand()*arena_size(2) ];
            robots(i).vel = [ speed, 0 ];
            robots(i).id  = i;
        end
        
        % run until all have passed through and stopped at the circle
        time_steps = 0;
        while time_steps < steps
            time_steps = time_steps + 1;
            for i = 1:num_robots
                if robots(i).pos(1) < wall_x - 2
                    % steer up/down to find the opening
                    if robots(i).pos(2) < hole_y(1) || robots(i).pos(2) > hole_y(2)
                        dir = sign( mean(hole_y) - robots(i).pos(2) );
                        robots(i).vel = [ 0, dir * speed ];
                    else
                        robots(i).vel = [ speed, 0 ];
                    end
                elseif robots(i).pos(1) > wall_x
                    % move toward its spot on the circle
                    theta = 2*pi*(i-1)/num_robots;
                    target = circle_center + circle_radius*[cos(theta),sin(theta)];
                    robots(i).pos = robots(i).pos + 0.05*(target - robots(i).pos);
                    robots(i).vel = [0,0];
                end
                robots(i).pos = robots(i).pos + robots(i).vel*dt;
            end
            
            % check if everyone stopped
            all_vels = vertcat(robots.vel);
            if all(all_vels(:,1)==0 & all_vels(:,2)==0)
                break
            end
        end
        
        results_gap(g,trial) = time_steps;
    end
end

% compute statistics
mean_gap = mean(results_gap, 2);
std_gap  = std(results_gap, 0, 2);

% plot
figure;
errorbar(gap_sizes, mean_gap, std_gap, 's-', 'LineWidth',2, 'MarkerSize',8);
xlabel('Wall‐Opening Height');
ylabel('Average Time Steps to Form Circle');
title('Formation Time vs. Opening Size (10 Robots)');
grid on;