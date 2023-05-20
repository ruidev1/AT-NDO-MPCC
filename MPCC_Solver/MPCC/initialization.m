%% Define starting position
% startIdx = 40; %point (in terms of track centerline array) allong the track 
startIdx = 1;
% where the car starts, on the center line, aligned with the track, driving
% straight with vx0
%since the used bicycle model is not well defined for slow velocities use vx0 > 0.5
if CarModel == "ORCA"
    vx0 = 0;
elseif CarModel == "FullSize"
    vx0 = 15;
end

% initial observer state
% global zdot;
% global z;
% global dhat;
% 
% zdot = zeros(3, 1);
% z = zeros(3, 1);
% dhat = zeros(3, 1);

% find theta that coresponds to the 10th point on the centerline
[theta, ~] = findTheta([track.center(1,startIdx),track.center(2,startIdx)],track.center,traj.ppx.breaks,trackWidth,startIdx);

x0 = [track.center(1,startIdx),track.center(2,startIdx),... % point on centerline
      atan2(ppval(traj.dppy,theta),ppval(traj.dppx,theta)),... % aligned with centerline
      vx0 ,0,0,theta]'; %driving straight with vx0, and correct theta progress
    
% the find theta function performs a local search to find the projection of
% the position onto the centerline, therefore, we use the start index as an
% starting point for this local search
last_closestIdx = startIdx;
cycle_flag = 0;
%% First initial guess
x = repmat(x0,1,N+1); % all points identical to current measurment
% first inital guess, all points on centerline aligned with centerline
% spaced as the car would drive with vx0
for i = 2:N+1
    theta_next = x(ModelParams.stateindex_theta,i-1)+Ts*vx0;
    phi_next = atan2(ppval(traj.dppy,theta_next),ppval(traj.dppx,theta_next));
    % phi_next can jump by two pi, make sure there are no jumps in the
    % initial guess
    if (x(ModelParams.stateindex_phi,i-1)-phi_next) < -pi
        phi_next = phi_next-2*pi;
    end
    if (x(ModelParams.stateindex_phi,i-1)-phi_next) > pi
        phi_next = phi_next+2*pi;
    end
    x(:,i) = [ppval(traj.ppx,theta_next),ppval(traj.ppy,theta_next),... % point on centerline
              phi_next,... % aligned with centerline
              vx0 ,0,0,theta_next]'; %driving straight with vx0, and correct theta progress
end

u = zeros(3,N); % zero inputs
uprev = zeros(3,1); % last input is zero

%% Ohter cars
Y = ObstacelsState(track,traj,trackWidth,n_cars);

if size(Y,2) ~= n_cars-1
    error('n_cars and the number of obstacles in "Y" does not match')
end

dhat = zeros(3, 1);
% dhat(1:1,1:simN)=0;
%% initializtion
% solve problem 5 times without applying input
% inspiered by sequential quadratic programming (SQP)
for i = 1:5
    % formulate MPCC problem and solve it
    Iter_damping = 0.5; % 0 no damping
    [x_up, u_up, b, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev, dhat);
    x = Iter_damping*x + (1-Iter_damping)*x_up;
    u = Iter_damping*u + (1-Iter_damping)*u_up;

    if plotOn == 1
        % plot predictions
        PlotPrediction(x,u,b,Y,track,track2,traj,MPC_vars,ModelParams)
    end
end

%% Initialize logging arrays
X_log = zeros(nx*(N+1),simN);
U_log = zeros(3*N,simN);
B_log = zeros(4*N,simN);
eC_log = zeros(2, simN);
v_log = zeros(2, simN);
qpTime_log = zeros(1,simN);
length_log = zeros(1, simN);

% observer state
z = zeros(3, 1);
% z = 0;
% z_dot = zeros(3, 1);
% dhat_log = zeros(3, simN);
dhat_log = zeros(3, simN);
zdot_log = zeros(3, simN);
z_log = zeros(3, simN);

% dotz(1:1,1:simN)=0;
% z(1:1,1:simN)=0;
% zdot = zeros(1, simN);
% 
% dhat(1:1,1:simN)=0;
% simulation times
j = 1;