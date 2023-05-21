% this script is used to run different MPCC scenarios with different
% parameters to compare different parameter settings performance

%% Clear everything
clc
clear
% close all

%% Add folders to the path
addpath(genpath('MPCC_Solver'));                  % MPCC control files

%% Load Parameters
CarModel = 'ORCA';
% CarModel = 'FullSize';

MPC_vars = getMPC_vars(CarModel);
ModelParams=getModelParams(MPC_vars.ModelNo);
% choose optimization interface options: 'Yalmip','CVX','hpipm','quadprog'
MPC_vars.interface = 'hpipm';

nx = ModelParams.nx;
nu = ModelParams.nu;
N = MPC_vars.N;
Ts = MPC_vars.Ts;

% here to change the MPC weight parameters
% MPC_vars.qC = 0.1;
% MPC_vars.qL = 10;

%% import an plot track
% use normal ORCA Track
load Tracks/track2.mat
% use RCP track
% load Tracks/trackMobil.mat
% track2 = trackMobil;
% shrink track by half of the car widht plus safety margin
% TODO implement orientation depending shrinking in the MPC constraints
safteyScaling = 1.5;
[track,track2] = borderAdjustment(track2,ModelParams,safteyScaling);
trackWidth_out = norm(track2.inner(:,1)-track2.outer(:,1));
trackWidth = norm(track.inner(:,1)-track.outer(:,1));
% plot shrinked and not shrinked track 
figure(1);
plot(track.outer(1,:),track.outer(2,:),'r')
hold on
plot(track.inner(1,:),track.inner(2,:),'r')
plot(track2.outer(1,:),track2.outer(2,:),'k')
plot(track2.inner(1,:),track2.inner(2,:),'k')

%% Simulation lenght and plotting
% simN = 500;
simN = 400;
%0=no plots, 1=plot predictions
plotOn = 1;
%0=real time iteration, 1=fixed number of QP iterations, 2=fixed number of damped QP iterations
QP_iter = 2;
% number of cars 
% there are two examples one with no other cars and one with 4 other cars
% (inspired by the set up shown in the paper)
n_cars = 1; % no other car
% n_cars = 5; % 4 other cars

%% Fit spline to track
% TODO spline function only works with regular spaced points.
% Fix add function which given any center line and bound generates equlally
% space tracks.
[traj, borders] =splinify(track);
tl = traj.ppy.breaks(end);

% store all data in one struct
TrackMPC = struct('traj',traj,'borders',borders,'track_center',track.center,'tl',tl);

%% values test
curvatures = zeros(1, 500);
for t = 1: 500
    theta = t * tl / 500;
    curvatures(t) = get_curvature(traj, theta);
end
max_curvature = max(curvatures);
%% Initial Run
% initialization;
% run_simulation;
% 
% % plot
% % PlotInfo(eC_log, v_log, simN, 0.02, 'qC=0.1');
% PlotInfo(eC_log, v_log, simN, 0.02, 'qVtheta=0.02');

%% Fix Weight Run
eC_max = trackWidth / 2;
reward = zeros(3, 1);
reward_step = zeros(3, simN);
slope = 10;
intercept = -3;

initialization;

% MPC_vars.qC = 10;
% MPC_vars.qVtheta= 1;
% MPC_vars.qL= 500;

times = 1;
run_simulation;
PlotInfo(eC_log, v_log, simN, 0.02, 'qC=0.001');

reward(1) = sum(reward_step(times,:));
% PlotInfo(eC_log, v_log, simN, 0.02, 'qVtheta=0.002');

figure(8);
% plot(track.outer(1,:),track.outer(2,:),'g', 'LineWidth',2)
% plot(track.inner(1,:),track.inner(2,:),'g', 'LineWidth',2)
plot(track2.outer(1,:),track2.outer(2,:),'k', 'LineWidth',2)
hold on
plot(track2.inner(1,:),track2.inner(2,:),'k', 'LineWidth',2)
plot(X_log(1,:),X_log(2,:),'b', 'LineWidth',1.8)
% scatter(X_log(1,:), X_log(2,:), [], X_log(4,:), 'filled', 'o', 'sizedata', 15);
% colormap(jet);
% colorbar;
if ~isempty(Y)
    for i=1:size(Y,2)
        carBox(Y(:,i),0.025,0.05)
    end
end
xlabel('X [m]')
ylabel('Y [m]')
axis equal
set(gca,'FontSize',12)
% hold off

figure(9);
% plot(track.outer(1,:),track.outer(2,:),'g', 'LineWidth',2)
% plot(track.inner(1,:),track.inner(2,:),'g', 'LineWidth',2)
plot(track2.outer(1,:),track2.outer(2,:),'k', 'LineWidth',2)
hold on
plot(track2.inner(1,:),track2.inner(2,:),'k', 'LineWidth',2)
% plot(X_log(1,:),X_log(2,:),'r', 'LineWidth',1.8)
scatter(X_log(1,:), X_log(2,:), [], X_log(4,:), 'filled', 'o', 'sizedata', 15);
colormap(jet);
colorbar;
if ~isempty(Y)
    for i=1:size(Y,2)
        carBox(Y(:,i),0.025,0.05)
    end
end
xlabel('X [m]')
ylabel('Y [m]')
axis equal
set(gca,'FontSize',12)

figure(11);
% simTimeSequence = 0:env.MPC_vars.Ts:env.MPC_vars.Ts * (400-1);
% plot(eC_log(1,:),eC_log(2,:),'b', 'LineWidth',2, 'DisplayName', 'MPCC');
plot(eC_log(1,:),eC_log(2,:),'m', 'LineWidth',2, 'DisplayName', 'NDO-MPCC');
hold on
xlabel('Driving Length [m]')
ylabel('Lateral Error [m]')
legend
set(gca,'FontSize',12)

figure(16);
simTimeSequence = 0:MPC_vars.Ts:MPC_vars.Ts * (400-1);
plot(simTimeSequence,eC_log(1,:),'b', 'LineWidth',2, 'DisplayName', 'MPCC');
% plot(simTimeSequence,eC_log(1,:),'m', 'LineWidth',2, 'DisplayName', 'NDO-MPCC');
hold on
xlabel('Time [sec]')
ylabel('Driving Length [m]')
legend
set(gca,'FontSize',12)
