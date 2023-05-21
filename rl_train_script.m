clear
% close all
clc
try
    nnet.internal.cnngpu.reluForward(1);
catch ME
end

addpath(genpath('MPCC_Solver'));

simN = 100;
isTrain = false;

env = MPCC_Env;
% validateEnvironment(env)

obsInfo = env.getObservationInfo();
actInfo = env.getActionInfo();
numObservations = 6;
numActions = 4;

% numObs = 6;
% numAct = 1;

rng(3);

% create_DDPG_agent;
% create_PPO_agent;
% create_SAC_agent;

maxepisodes = 5000;
maxsteps = 400;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'ScoreAveragingWindowLength',5,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',1000,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',449,...
    "SaveAgentDirectory",pwd + "/savedAgents/Agents");
trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode = 'async';

if isTrain
%     env.plotOn = 1;
    trainingStats = train(agent,env,trainOpts);
else
    load(pwd + "/savedAgents/Agent4999", "saved_agent");
    agent = saved_agent;
    env.plotOn = 1;
    env.isTrain = false;
    Observation = env.reset();
    simOpts = rlSimulationOptions("MaxSteps",400);
    experience = sim(env, agent, simOpts);
    Reward_log = env.reward_log;
    total_reward = sum(Reward_log);
    load('Info_log.mat');
    Info_log = out;
    PlotInfo(Info_log(1:2, :), Info_log(3:4, :), simN, 0.02, 'RL');

    figure(8);
%     plot(env.track.outer(1,:),env.track.outer(2,:),'g', 'LineWidth',2)
%     plot(env.track.inner(1,:),env.track.inner(2,:),'g', 'LineWidth',2)
    plot(env.track2.outer(1,:),env.track2.outer(2,:),'k', 'LineWidth',2)
    hold on
    plot(env.track2.inner(1,:),env.track2.inner(2,:),'k', 'LineWidth',2)
    plot(env.X_log(1,:),env.X_log(2,:),'r', 'LineWidth',1.8)
%     scatter(X_log(1,:), X_log(2,:), [], X_log(4,:), 'filled', 'o', 'sizedata', 30);
%     colormap(jet);
%     colorbar;
    xlabel('X [m]')
    ylabel('Y [m]')
    axis equal
    set(gca,'FontSize',12)
    
    figure(9);
    % plot(track.outer(1,:),track.outer(2,:),'g', 'LineWidth',2)
    % plot(track.inner(1,:),track.inner(2,:),'g', 'LineWidth',2)
    plot(env.track2.outer(1,:),env.track2.outer(2,:),'k', 'LineWidth',2)
    hold on
    plot(env.track2.inner(1,:),env.track2.inner(2,:),'k', 'LineWidth',2)
    % plot(X_log(1,:),X_log(2,:),'r', 'LineWidth',1.8)
    scatter(env.X_log(1,:), env.X_log(2,:), [], env.X_log(4,:), 'filled', 'o', 'sizedata', 15);
    colormap(jet);
    colorbar;
    xlabel('X [m]')
    ylabel('Y [m]')
    axis equal
    set(gca,'FontSize',12)
    
    f = figure(10);
    simTimeSequence = 0:env.MPC_vars.Ts:env.MPC_vars.Ts * (400-1);
    subplot(221)
    scatter(simTimeSequence, env.Params_log(1, :), '.', 'SizeData', 10);
    xlabel('Time (sec)')
    ylabel('qC')
    set(gca,'FontSize',12)
    subplot(222)
    scatter(simTimeSequence, env.Params_log(2, :), '.', 'SizeData', 10);
    xlabel('Time (sec)')
    ylabel('qVtheta')
    set(gca,'FontSize',12)
    subplot(223)
    scatter(simTimeSequence, env.Params_log(3, :), '.', 'SizeData', 10);
    xlabel('Time (sec)')
    ylabel('qL')
    set(gca,'FontSize',12)
    subplot(224)
    scatter(simTimeSequence, env.Params_log(4, :), '.', 'SizeData', 10);
    xlabel('Time (sec)')
    ylabel('qOmega')
    set(gca,'FontSize',12)
%     exportgraphics(f, 'parameters log.pdf','ContentType','vector');
    
    figure(16);
    simTimeSequence = 0:env.MPC_vars.Ts:env.MPC_vars.Ts * (400-1);
    plot(simTimeSequence,env.length_log(1,:), 'Color', '#EDB120', 'LineStyle', '--', 'LineWidth',2, 'DisplayName', 'AT-MPCC');
%     plot(simTimeSequence,env.length_log(1,:),'r', 'LineWidth',2, 'DisplayName', 'AT-NDO-MPCC');
    hold on
    xlabel('Time [sec]')
    ylabel('Driving Length [m]')
    legend
    set(gca,'FontSize',12)
    
end
