clc; clear;close all;

% Create shared environment
env = RLFormationSharedEnv(1);  % use n robots
obs = reset(env);

% Use only the first observation/action info (they're all the same)
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);

% Create single shared agent
dnn = [
    featureInputLayer(obsInfo(1).Dimension(1), 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(128, 'Name', 'fc1')
    reluLayer('Name', 'relu1')
    fullyConnectedLayer(128, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(numel(actInfo(1).Elements), 'Name', 'output')
];

opts = rlRepresentationOptions('LearnRate', 1e-3, 'GradientThreshold', 1);
qRep = rlQValueRepresentation(dnn, obsInfo(1), actInfo(1), 'Observation', {'state'}, opts);

% Epsilon-greedy exploration settings
expOpts = rl.option.EpsilonGreedyExploration;
expOpts.Epsilon = 0.9;
expOpts.EpsilonMin = 0.01;
expOpts.EpsilonDecay = 1e-5;

% Agent options
agentOpts = rlDQNAgentOptions(...
    'SampleTime', 0.1, ...
    'UseDoubleDQN', true, ...
    'TargetSmoothFactor', 1e-3, ...
    'ExperienceBufferLength', 1e5, ...
    'DiscountFactor', 0.99, ...
    'MiniBatchSize', 64, ...
    'EpsilonGreedyExploration', expOpts);

% Create the agent
agent = rlDQNAgent(qRep, agentOpts);

% Training options (5000 episodes)
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', 1000, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'EpisodeCount', ...
    'StopTrainingValue', 1000);

% Train the agent
trainingStats = train(agent, env, trainOpts);

% Save the trained agent
save('trainedSharedAgentSwarm_5000eps.mat', 'agent', 'trainingStats');
runTrainedAgent