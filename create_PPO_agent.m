% actInfo = env.getActionInfo();
% obsInfo = env.getObservationInfo();
% 
% rng(0)
%% 

criticNet = [
    featureInputLayer(prod(obsInfo.Dimension))
    fullyConnectedLayer(100)
    reluLayer
    fullyConnectedLayer(1)];

criticNet = dlnetwork(criticNet);
summary(criticNet)

critic = rlValueFunction(criticNet,obsInfo, 'UseDevice','gpu');

getValue(critic,{rand(obsInfo.Dimension)})

%% 
% Define common input path layer
commonPath = [ 
    featureInputLayer(prod(obsInfo.Dimension),Name="comPathIn")
    fullyConnectedLayer(100)
    reluLayer
    fullyConnectedLayer(1,Name="comPathOut") ];

% Define mean value path
meanPath = [
    fullyConnectedLayer(15,Name="meanPathIn")
    reluLayer
    fullyConnectedLayer(prod(actInfo.Dimension));
    tanhLayer;
    scalingLayer(Name="meanPathOut",Scale=actInfo.UpperLimit) ];

% Define standard deviation path
sdevPath = [
    fullyConnectedLayer(15,'Name',"stdPathIn")
    reluLayer
    fullyConnectedLayer(prod(actInfo.Dimension));
    softplusLayer(Name="stdPathOut") ];

% Add layers to layerGraph object
actorNet = layerGraph(commonPath);
actorNet = addLayers(actorNet,meanPath);
actorNet = addLayers(actorNet,sdevPath);

% Connect paths
actorNet = connectLayers(actorNet,"comPathOut","meanPathIn/in");
actorNet = connectLayers(actorNet,"comPathOut",'stdPathIn/in');

% Plot network 
plot(actorNet)
% Convert to dlnetwork and display number of weights
actorNet = dlnetwork(actorNet);
summary(actorNet)

actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    'ActionMeanOutputNames',"meanPathOut",...
    'ActionStandardDeviationOutputNames',"stdPathOut",...
    'ObservationInputNames',"comPathIn", ...
    'UseDevice','gpu');

getAction(actor,{rand(obsInfo.Dimension)})

%% 
agent = rlPPOAgent(actor,critic)

agent.AgentOptions.ExperienceHorizon = 1024;
agent.AgentOptions.DiscountFactor = 0.95;

agent.AgentOptions.CriticOptimizerOptions.LearnRate = 8e-3;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold = 1;
agent.AgentOptions.ActorOptimizerOptions.LearnRate = 8e-3;
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold = 1;

criticOpts = rlOptimizerOptions( ...
    'LearnRate',8e-3,'GradientThreshold',1);

getAction(agent,{rand(obsInfo.Dimension)})