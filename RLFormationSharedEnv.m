classdef RLFormationSharedEnv < rl.env.MATLABEnvironment
    properties
        NumRobots
        ArenaSize = [100, 50]
        WallX
        HoleYRange = [20, 30] %change back for observation, works fro 5,45 so far.
        CircleCenter
        CircleRadius = 10
        Speed = 10.0
        dt = 0.1
        Steps = 1000
        StepCount
        Robots
        reward=0
        CircleRewardGiven = false % new 

                % handles used for drawing
        Fig
        Ax
        HBots
    end

    properties(Access = protected)
        IsDone = false; % check isdone upper and lower case
    end

    methods

    function this = RLFormationSharedEnv(numRobots)
    if nargin < 1
        numRobots = 10;
    end

    obsInfo = rlNumericSpec([2*numRobots 1], 'LowerLimit', 0, 'UpperLimit', 100);
    obsInfo.Name = 'positions';

    actInfo = rlFiniteSetSpec(0:3);
    actInfo.Name = 'shared_action';

    
    this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);

    this.NumRobots = numRobots;
    this.WallX = this.ArenaSize(1)/2;
    %this.HoleYRange = [5, 45];
    this.CircleCenter = [this.WallX + 20, mean(this.HoleYRange)];
end

function initialObs = reset(this)
    this.StepCount = 0;
    this.IsDone = false;
    this.reward=0;

    this.Robots = struct('pos', {}, 'vel', {}, 'id', {});
    for i = 1:this.NumRobots
        this.Robots(i).pos = [rand()*this.WallX/2, rand()*this.ArenaSize(2)];
        this.Robots(i).vel = [this.Speed, 0];
        this.Robots(i).id = i;
    end

    initialObs = this.getObservation();
    
    this.plotEnvironment();
end

        function [obs, reward, isDone, info] = step(this, action)
            %disp("👣 Inside STEP function");
            
            this.StepCount = this.StepCount + 1;
            %whos
            reward = 0;%this.reward;
            for i = 1:this.NumRobots
                switch action
                    case 1
                        this.Robots(i).vel = [this.Speed, 0];
                    case 2
                        this.Robots(i).vel = [0, this.Speed];
                    case 3
                        this.Robots(i).vel = [0, -this.Speed];
                    otherwise
                        this.Robots(i).vel = [0, 0];
                end

                x = this.Robots(i).pos(1);
                y = this.Robots(i).pos(2);
                
                if abs(x+this.Robots(i).vel(1)* this.dt-this.WallX)<2
                    if y+this.Robots(i).vel(2)* this.dt < this.HoleYRange(1) || y+this.Robots(i).vel(2)* this.dt > this.HoleYRange(2)
                        % this.IsDone= true;
                        % reward = reward - 100; % add a rewrds for circle formnig
                        this.Robots(i).pos = this.Robots(i).pos - this.Robots(i).vel * this.dt;
                    else
                        % reward = reward + 100;                       
                    end
                end
                if y+this.Robots(i).vel(2)* this.dt<0
                    % reward=reward-100;
                    % this.IsDone= true;
                    this.Robots(i).pos = this.Robots(i).pos - this.Robots(i).vel * this.dt;
                end
                if y+this.Robots(i).vel(2)* this.dt>this.ArenaSize(2)
                    % reward=reward-100;
                    % this.IsDone= true;
                    this.Robots(i).pos = this.Robots(i).pos - this.Robots(i).vel * this.dt;
                end
%                 if x < this.WallX - 2
%                     if y < this.HoleYRange(1) || y > this.HoleYRange(2)
%                         this.Robots(i).vel = [0, sign(mean(this.HoleYRange) - y) * this.Speed];
%                     else
%                         this.Robots(i).vel = [this.Speed, 0];
%                     end
%                 elseif x > this.WallX
%                     theta = 2*pi*(i-1)/this.NumRobots;
%                     target = this.CircleCenter + this.CircleRadius * [cos(theta), sin(theta)];
%                     this.Robots(i).pos = this.Robots(i).pos + 0.05 * (target - this.Robots(i).pos);
%                     this.Robots(i).vel = [0, 0];
%                 end

                this.Robots(i).pos = this.Robots(i).pos + this.Robots(i).vel * this.dt;
            end

            % Calculate reward + circle reward logic 
            
             for i = 1:this.NumRobots
                for j = i+1:this.NumRobots
                    if norm(this.Robots(i).pos - this.Robots(j).pos) < 2
                        reward = reward - 100;
                    end
                end
            end

            formationReward = 0;
            threshold = 3*this.Speed*this.dt;
            positions = reshape([this.Robots.pos], 2, []).';
            distances = vecnorm(positions - this.CircleCenter, 2, 2);
            onRightSide = positions(:,1) > this.WallX;
            inCircle = abs(distances - this.CircleRadius) < threshold;

            if all(onRightSide & inCircle) && ~this.CircleRewardGiven
                formationReward = 100;%-sum(abs(distances - this.CircleRadius))*this.StepCount/this.Steps/this.ArenaSize(1);
                %this.CircleRewardGiven = true;
            end
            % if x<this.WallX
            %     reward=reward-10*sqrt((x-this.WallX).^2+(y-mean(this.HoleYRange))^2);
            % end
            reward = reward + formationReward;
            reward = reward - 0.1;

            isDone = this.StepCount >= this.Steps;
            this.IsDone = isDone;

            obs = this.getObservation();
            info = struct();
        % === Final output guard ===
        %disp("✅ About to return from step()");
        %whos
        if ~exist('obs', 'var'), obs = this.getObservation(); end
        if ~exist('reward', 'var'), reward = 0; end
        if ~exist('isDone', 'var'), isDone = false; end
        if ~exist('info', 'var'), info = struct(); end
        
        this.reward=reward;
        this.plotEnvironment();
        
        end

        function obs = getObservation(this)
            obs = reshape([this.Robots.pos], [], 1);
        end
        
        function plotEnvironment(this)
            % Create the figure/axes the first time ---------------------
            if isempty(this.Fig) || ~isvalid(this.Fig)
                this.Fig = figure("Name","RLFormationEnv","Color","w");
                this.Ax  = axes(this.Fig);  hold(this.Ax,"on");  axis equal
                xlim(this.Ax,[0 this.ArenaSize(1)]);
                ylim(this.Ax,[0 this.ArenaSize(2)]);
                xlabel(this.Ax,"X"); ylabel(this.Ax,"Y");
                title(this.Ax,num2str(this.reward));
                % static wall and hole
                plot(this.Ax,[this.WallX this.WallX],[0 this.HoleYRange(1)], "k-", "LineWidth",2);
                plot(this.Ax,[this.WallX this.WallX],[this.HoleYRange(2) this.ArenaSize(2)],"k-", "LineWidth",2);
                
                % target circle
                % target circle (toolbox-free)
                th = linspace(0, 2*pi, 150);
                plot(this.Ax,this.CircleCenter(1) + this.CircleRadius*cos(th),this.CircleCenter(2) + this.CircleRadius*sin(th),"--");

                % scatter plot for robots (start empty)
                this.HBots = scatter(this.Ax, nan, nan, 60, "filled");
            end

            % Update robot positions every call -------------------------
            pos = reshape([this.Robots.pos], 2, []).';   % 10x2 → [x y]
            this.HBots.XData = pos(:,1);
            this.HBots.YData = pos(:,2);
            drawnow limitrate
        end


    end
end