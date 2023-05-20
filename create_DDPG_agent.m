% numObs = 1;
% numActions = 2;
% 
% env = MPCC_Env;
% %% create critic
% statePath = [
%     featureInputLayer(numObs,'Normalization','none','Name','observation')
%     fullyConnectedLayer(400,'Name','CriticStateFC1')
%     reluLayer('Name', 'CriticRelu1')
%     fullyConnectedLayer(300,'Name','CriticStateFC2')];
% actionPath = [
%     featureInputLayer(numActions,'Normalization','none','Name','action')
%     fullyConnectedLayer(300,'Name','CriticActionFC1','BiasLearnRateFactor',0)];
% commonPath = [
%     additionLayer(2,'Name','add')
%     reluLayer('Name','CriticCommonRelu')
%     fullyConnectedLayer(1,'Name','CriticOutput')];
% 
% criticNetwork = layerGraph();
% criticNetwork = addLayers(criticNetwork,statePath);
% criticNetwork = addLayers(criticNetwork,actionPath);
% criticNetwork = addLayers(criticNetwork,commonPath);
%     
% criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
% criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');
% criticNetwork = dlnetwork(criticNetwork);
% 
% figure
% plot(layerGraph(criticNetwork))
% 
% criticOpts = rlOptimizerOptions('LearnRate',1e-03,'GradientThreshold',1);
% 
% obsInfo = getObservationInfo(env);
% actInfo = getActionInfo(env);
% critic = rlQValueFunction(criticNetwork,obsInfo,actInfo,'ObservationInputNames','observation','ActionInputNames','action');
% 
% %% create actor
% actorNetwork = [
%     featureInputLayer(numObs,'Normalization','none','Name','observation')
%     fullyConnectedLayer(400,'Name','ActorFC1')
%     reluLayer('Name','ActorRelu1')
%     fullyConnectedLayer(300,'Name','ActorFC2')
%     reluLayer('Name','ActorRelu2')
%     fullyConnectedLayer(numActions,'Name','ActorFC3')
%     tanhLayer('Name','ActorTanh')
%     scalingLayer('Name','ActorScaling','Scale',max(actInfo.UpperLimit))];
% actorNetwork = dlnetwork(actorNetwork);
% 
% actorOpts = rlOptimizerOptions('LearnRate',1e-04,'GradientThreshold',1);
% 
% actor = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo);
% 
% figure
% plot(layerGraph(actorNetwork))
% 
% %% create agent
% agentOpts = rlDDPGAgentOptions(...
%     'SampleTime',env.MPC_vars.Ts,...
%     'CriticOptimizerOptions',criticOpts,...
%     'ActorOptimizerOptions',actorOpts,...
%     'ExperienceBufferLength',1e6,...
%     'DiscountFactor',0.99,...
%     'MiniBatchSize',128);
% agentOpts.NoiseOptions.Variance = 0.6;
% agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
% 
% agent = rlDDPGAgent(actor,critic,agentOpts);

%% create simple DDPG agent
statePath = [imageInputLayer([numObservations 1 1],'Normalization','none','Name','state')
            fullyConnectedLayer(300,'Name','CriticStateFC1')
            reluLayer('Name','CriticStateRelu1')
            fullyConnectedLayer(300,'Name','CriticStateFC2')
            reluLayer('Name','CriticStateRelu2')
            fullyConnectedLayer(100,'Name','CriticStateFC3')
            ];
actionPath = [imageInputLayer([numActions 1 1],'Normalization','none','Name','action')
              fullyConnectedLayer(300,'Name','CriticActionFC1')
              reluLayer('Name','CriticActionRelu1')
              fullyConnectedLayer(100,'Name','CriticActionFC2')
               ];
commonPath = [concatenationLayer(1,2,'Name','concat')
             quadraticLayer('Name','quadratic')
             fullyConnectedLayer(1,'Name','StateValue','BiasLearnRateFactor',0,'Bias',0)];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);

% criticNetwork = connectLayers(criticNetwork,'state','concat/in1');
criticNetwork = connectLayers(criticNetwork, 'CriticStateFC3', 'concat/in1');
% criticNetwork = connectLayers(criticNetwork,'action','concat/in2');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC2','concat/in2');

criticOpts = rlRepresentationOptions('LearnRate',5e-3,'GradientThreshold',1);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'state'},'Action',{'action'},criticOpts);

% actor
actorNetwork = [
    imageInputLayer([numObservations 1 1],'Normalization','none','Name','state')
    fullyConnectedLayer(400,'Name','CriticActionFC1')
    reluLayer('Name','CriticActionRelu1')
    fullyConnectedLayer(400,'Name','CriticActionFC2')
    reluLayer('Name','CriticActionRelu2')
    fullyConnectedLayer(300,'Name','CriticActionFC3')
    reluLayer('Name','CriticActionRelu3')
    fullyConnectedLayer(300,'Name','CriticActionFC4')
    reluLayer('Name','CriticActionRelu4')
    fullyConnectedLayer(300,'Name','CriticActionFC5')
    reluLayer('Name','CriticActionRelu5')
    fullyConnectedLayer(numActions,'Name','action')];

actorOpts = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'state'},'Action',{'action'},actorOpts);

% DDPG agent
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',env.MPC_vars.Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',32);
% agentOpts.NoiseOptions.Variance = 2;
% agentOpts.NoiseOptions.VarianceDecayRate = 1e-6;
opt.NoiseOptions.StandardDeviation = [1 0.1];
opt.NoiseOptions.StandardDeviationDecayRate = 1e-6;

agent = rlDDPGAgent(actor,critic,agentOpts);