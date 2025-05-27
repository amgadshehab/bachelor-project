%% runTrainedAgent.m
% Roll out a trained agent in RLFormationSharedEnv and (optionally) record
% the on-screen animation to MP4.  Requires your env’s reset/step to call
% plotEnvironment so a figure is updated each time.

clear; clc; close all;

%% ---------------- USER SETTINGS -------------------------------------
agentFile     = "trainedSharedAgentSwarm_2500.mat";   % .mat file
agentVarName  = "agent";                                 % variable inside
numEpisodes   = 6;                                       % roll-outs
recordVideo   = true;                                    % true → save MP4
videoFolder   = replace('agentVideos '+string(datetime),':','_');         % output dir
rng(0);                                                  % reproducibility
% ---------------------------------------------------------------------

%% -------- Load the trained agent ------------------------------------
assert(isfile(agentFile), "Could not find %s", agentFile);
A      = load(agentFile, agentVarName, 'trainingStats');
agent  = A.(agentVarName);


%% -------- Prepare video folder --------------------------------------
if recordVideo && ~isfolder(videoFolder)
    mkdir(videoFolder);
end

%% Plot all training results
figure;plot(A.trainingStats.EpisodeQ0);xlabel('EpisodeNumber');ylabel('Q_0');
saveas(gcf,[videoFolder+'/Q0'],'fig');saveas(gcf,[videoFolder+'/Q0'],'jpg');

figure;plot(A.trainingStats.EpisodeReward,'b-');xlabel('EpisodeNumber');ylabel('Episode reward, Average reward');hold on;
plot(A.trainingStats.AverageReward,'k-','LineWidth',2);
saveas(gcf,[videoFolder+'/Reward'],'fig');saveas(gcf,[videoFolder+'/Reward'],'jpg');

figure;plot(A.trainingStats.EvaluationStatistic);xlabel('EpisodeNumber');ylabel('Evaluation statistic');
saveas(gcf,[videoFolder+'/EvaluationStatistic'],'fig');saveas(gcf,[videoFolder+'/EvaluationStatistic'],'jpg');

figure;plot(A.trainingStats.AverageSteps);xlabel('EpisodeNumber');ylabel('Average Steps');
saveas(gcf,[videoFolder+'/AverageSteps'],'fig');saveas(gcf,[videoFolder+'/AverageSteps'],'jpg');

figure;plot(A.trainingStats.TotalAgentSteps);xlabel('EpisodeNumber');ylabel('TotalAgentSteps');
saveas(gcf,[videoFolder+'/TotalAgentSteps'],'fig');saveas(gcf,[videoFolder+'/TotalAgentSteps'],'jpg');

%% new
%% ====== Gap-crossing sweep (ignore circle) ==========================
gapSizes        = [5 10 15 20];   % wall-opening heights to test
episodesPerGap  = 5;              % repeats for averaging
crossTimeSteps  = NaN(numel(gapSizes), episodesPerGap);

for g = 1:numel(gapSizes)
    gap = gapSizes(g);
    fprintf("\n--- Gap %2d ---\n", gap);

    for ep = 1:episodesPerGap
        % 1. fresh env with this gap size
        env = RLFormationSharedEnv(1);      % 2 = #robots (change if needed)
        ctr = mean(env.HoleYRange);
        env.HoleYRange = ctr + [-gap/2 gap/2];

        obs = reset(env);      t = 0;       crossed = false;

        % 2. step loop
        while ~crossed && t < env.Steps
            act = getAction(agent, obs);  if iscell(act), act = act{1}; end
            [obs, ~] = step(env, act);    % ignore reward/isDone

            t = t + 1;

            % 3. check crossing condition for **robot 1**
            x  = obs(1);                      % x-coord of first robot
            y  = obs(2);                      % y-coord             "
            inY = y > env.HoleYRange(1) && y < env.HoleYRange(2);
            if x > env.WallX && inY
                crossed = true;
            end
        end

        crossTimeSteps(g, ep) = t;     % 1000 if never crossed
    end
end

% ------------ plot: mean ± std vs. gap size --------------------------
meanT = mean(crossTimeSteps, 2);
stdT  = std(crossTimeSteps,  0, 2);

figure;
errorbar(gapSizes, meanT, stdT, 's-', 'LineWidth',1.5, ...
         'MarkerSize',8, 'CapSize',12);
xlabel("Wall-Opening Height");
ylabel("Average Steps to Cross Gap");
title(sprintf("Gap-Cross Time vs. Opening Size (%d Robot)", env.NumRobots));
grid on;

if recordVideo && ~isfolder(videoFolder), mkdir(videoFolder); end
saveas(gcf, fullfile(videoFolder,"GapCrossPlot.fig"));
saveas(gcf, fullfile(videoFolder,"GapCrossPlot.jpg"));
%% ====================================================================
%% new 

%% -------- Instantiate environment -----------------------------------
env = RLFormationSharedEnv(1);       % use 1 robots
obs = reset(env);

maxSteps = env.Steps;                % property you defined

%% ====================== EPISODE LOOP ================================
for ep = 1:numEpisodes
    obs       = reset(env);          % also triggers first draw
    epReward  = 0;

    %% ----- Optional recorder set-up ---------------------------------
    recObj  = [];   useRLP = false;
    if recordVideo
        if exist("rlVideoPlayer","file") == 2
            recObj = rlVideoPlayer( ...
                     "FileName", fullfile(videoFolder, ...
                     sprintf("Episode_%03d.mp4", ep)));
            useRLP = true;
        else
            fname  = fullfile(videoFolder, ...
                     sprintf("Episode_%03d_fallback.mp4", ep));
            recObj = VideoWriter(fname,"MPEG-4");
            open(recObj);
        end
        % write first frame
        frame = getframe(gcf);
        if useRLP, recObj.writeFrame(frame); else, writeVideo(recObj,frame); end
    end
    % -----------------------------------------------------------------

    %% --------- STEP LOOP -------------------------------------------
    for t = 1:maxSteps
        act = getAction(agent, obs);           % 0–3, maybe as {0}
        if iscell(act), act = act{1}; end      % unwrap if needed

        [nextObs, rwd, isDone] = step(env, act);  % env draws itself
        epReward = epReward + rwd;

        drawnow limitrate;                     % flush GUI

        % record frame if requested
        if recordVideo
            frame = getframe(gcf);
            if useRLP, recObj.writeFrame(frame); else, writeVideo(recObj,frame); end
        end

        if isDone
            break;
        end
        obs = nextObs;
    end

    %% ----- Close recorder ------------------------------------------
    if recordVideo
        if useRLP
            recObj.close();
        else
            close(recObj);
        end
    end
    % -----------------------------------------------------------------

    fprintf("Episode %d finished after %d steps | Reward = %.2f\n", ...
            ep, t, epReward);
end

disp("Rollouts complete.");