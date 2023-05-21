ji = j - 1;
% Difinition of disturbance input at 10s 
% global disturbance;
disturbance = zeros(3, simN);
% disturbance = zeros(1, simN);
disturbance(1, 1:simN) = 0.2;
disturbance(2, 1:simN) = 0.7;
disturbance(3, 1:simN) = 0.5;
% disturbance = 1;
dhat = 0;
dhat_test = 0;
%% Simulation
for i = 1: simN
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% MPCC-Call %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    decay = 0.001;
    if i > 1
        disturbance(:, i) = disturbance(:, i-1) - decay * disturbance(:, i-1) + 0.001 * wgn(3,1,0);
%         disturbance(:, i) = disturbance(:, i-1) - decay * disturbance(:, i-1);
    end
    d = disturbance(:, i);
%     d = 0;
    % augment state and inputs by shifting previus optimal solution
    [x,u] = augState(x,u,x0,MPC_vars,ModelParams,tl, d);

    %% NDO
    
%     xdot = fx_bicycle(0, x(:, 1), u(:, 1), ModelParams);
%     
%     % Disturbance input matrix
%     phi = x(3, 1);
%     omega = x(6, 1);
%     R = [cos(phi), -sin(phi), 0;
%          sin(phi), cos(phi), 0;
%          0, 0, 1];
%     R_dot = [-sin(phi) * omega, -cos(phi) * omega, 0;
%               cos(phi) * omega, -sin(phi) * omega, 0;
%               0, 0, 0];
%     % G = R.T;
%     
%     % zdot = zeros(3, 1);
%     % z = zeros(3, 1);
%     
% %     L = diag([100, 100, 100]);
%     L = 50;
%     
%     % dhat = zeros(3, 1);
%     
%     % % Difinition of disturbance input at 10s 
%     % d(1:N/2)   = 0;
%     % d(N/2+1:N) = 1*1;
%     
%     % Caluculation of observer and estimation of disturbance
% %     x2 = x(4:6, 1);
% %     f2x=xdot(4:6);
%     x2 = x(6, 1);
%     f2x=xdot(6);
% %     px=L * R * x2;
%     px = L * x2;
% %     zdot=-L *z - L *(R_dot*x2 + R * f2x + px);
%     zdot=-L *z - L *(f2x + px);
% %     [~,iniz]=ode45(@(t,z) -L *z - L *(R_dot*x2 + R * f2x + px),[0 Ts],z);
%     [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 Ts],z);
%     z=iniz(end,:)';
%     dhat=z+px;
%     dhat_log(:, i) = dhat;
%     disp(ModelParams.dhat)

    %  formulate MPCC problem and solve it
    if QP_iter == 0
        [x, u, b, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev, dhat_test);
        qpTime_log(i) = info.QPtime;
    elseif QP_iter == 1
        % doing multiple "SQP" steps
        for k = 1:2
            [x, u, b, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev, dhat_test);
            qpTime_log(i) = qpTime_log(i) + info.QPtime;
        end
    elseif QP_iter == 2
        % doing multiple damped "SQP" steps
        for k = 1:2
            Iter_damping = 0.75; % 0 no damping
            [x_up, u_up, b, exitflag,info] = optimizer_mpcc(TrackMPC,MPC_vars,ModelParams, n_cars, Y, x, u, x0, uprev, dhat_test);
            x = Iter_damping*x + (1-Iter_damping)*x_up;
            u = Iter_damping*u + (1-Iter_damping)*u_up;
            qpTime_log(i) = qpTime_log(i) + info.QPtime;
        end
    else
        error('invalid QP_iter value')
    end

    % NDO
%     xdot = fx_bicycle(0, x(:, 1), u(:, 1), ModelParams);
    xdot = fx_bicycle(0, x(:,1), u(:, 1), ModelParams);
    
    % Disturbance input matrix
%     phi = x(3, 1);
%     omega = x(6, 1);
    phi = x0(3);
    omega = x0(6);

    R = [cos(phi), -sin(phi), 0;
         sin(phi), cos(phi), 0;
         0, 0, 1];
    R_dot = [-sin(phi) * omega, -cos(phi) * omega, 0;
              cos(phi) * omega, -sin(phi) * omega, 0;
              0, 0, 0];
    % G = R.T;
    
    % zdot = zeros(3, 1);
    % z = zeros(3, 1);
    
%     L = diag([100, 100, 100]);
%     L = 0.8;
%     L = diag([1.2, 0.5, 0.8]);
    L = diag([1.2, 0.5, 0.2]);
    
    % dhat = zeros(3, 1);
    
    % % Difinition of disturbance input at 10s 
    % d(1:N/2)   = 0;
    % d(N/2+1:N) = 1*1;
    
    % Caluculation of observer and estimation of disturbance
    x2 = x(4:6, 1);
    f2x=xdot(4:6);
%     x2 = x(6, 1);
%     f2x=xdot(6);
%     px=L * R * x2;
%     px = L * x2;
% %     zdot=-L *z - L *(R_dot*x2 + R * f2x + px);
%     zdot=-L *z - L *(f2x + px);
% %     [~,iniz]=ode45(@(t,z) -L *z - L *(R_dot*x2 + R * f2x + px),[0 Ts],z);
%     [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 Ts],z);
%     z=iniz(end,:)';
%     dhat=z+px;
%     dhat_log(:, i) = dhat;

%     dhat_dot = -L * dhat + L * (d);
    x2_dot = fx_bicycle_disturbance(0, x(:, 1), u(:, 1), ModelParams, d);
    px = L * x2;
    dhat_test = z + px;
    dhat_log(:, i) = dhat_test;
    if i == 1
        dhat = z + px;
    end
    dhat_dot = -L * dhat + L * (x2_dot(4:6) - f2x);
    z_dot = -L *z - L *(f2x + px);
    px_dot = L * x2_dot(4:6);
    z_dot_test = dhat_dot - px_dot;
    dhat_dot_test = z_dot + px_dot;
%     dhat = dhat + dhat_dot * Ts;
    [~,inid]=ode45(@(t,dhat) -L * dhat + L * (x2_dot(6) - f2x),[0 Ts],dhat);
    dhat=inid(end,:)';
    z_direct = z + z_dot * Ts;
    [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 Ts],z);
    z=iniz(end,:)';
%     x2_test = x2 + x2_dot(6) * Ts;
    [~,inix]=ode45(@(t,x) fx_bicycle_disturbance(0, x, u(:, 1), ModelParams, d),[0 Ts],x(:, 1));
    x2_test = inix(end, 4:6)';

    px_appro = L * x2 + L * x2_dot(4:6) * Ts;
    px_fine_intergral = L * x2_test;

    z_should = dhat - px_appro;
%     z_dot = -L *z - L *(f2x + px);
%     z_dot_test = dhat_dot - px_dot;
%     dhat_dot_test = z_dot + px_dot;
%     px_test_direct = px + px_dot * Ts;
%     px_test = x2_test * L;
%     dhat_test = z + px_test;
%     dhat_test = z + px;
%     [~,inid]=ode45(@(t,dhat) -L * dhat + L * (x2_dot(6) - f2x),[0 Ts],dhat);
%     [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 Ts],z);
%     dhat=inid(end,:)';
%     z=iniz(end,:)';
%     dhat = z + px;
%     dhat_log(:, i) = dhat;

%     fx=A*x(:,i)+[0;-5*tanh(2*x(2,i))];
%     R = g2(2);
%     x2 = x(2, i);
%     F=fx + B * u(i);
%     f2x = F(2);
    
%     xdot = fx_bicycle(0, x(:,1), u(:, 1), ModelParams);
%     x2 = x(6, 1);
%     f2x=xdot(6);
% 
% %     px=L * inv(R) * x2;
%     px = L * x2;
%     
% 
% %     zdot(:, i+1)=-L *z(:, i) - L *(inv(R) * f2x + px);
%     [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 Ts],z(:, i));
%     z(:, i+1)=iniz(end,:)';
% %     z(:,i+1)=z(:,i)+zdot(:,i+1)*Ts;
%     dhat(:, i+1)=z(:, i+1)+px;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% simulate system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    x0 = SimTimeStep(x(:,1),u(:,1),Ts,ModelParams, d)';
    x0 = unWrapX0(x0);
    
    [ theta, last_closestIdx] = findTheta(x0,track.center,traj.ppx.breaks,trackWidth,last_closestIdx);
    x0(ModelParams.stateindex_theta) = theta;
    uprev = u(:,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% plotting and logging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if plotOn == 1
        PlotPrediction(x,u,b,Y,track,track2,traj,MPC_vars,ModelParams)
    end
    
    % log predictions and time
    X_log(:,i) = reshape(x,(N+1)*7,1);
    U_log(:,i) = reshape(u,(N)*3,1);
    B_log(:,i) = reshape(b,N*4,1);

    Xk = x0;
    length = Xk(end);
    if i > 1 && length < length_log(1, i-1)
        cycle_flag = 1;
    end
    if cycle_flag == 1
        length = length + tl;
    end
    
    x_phys = Xk(1);
    y_phys = Xk(2);
    theta_virt=mod(Xk(end),traj.ppx.breaks(end));
    [eC, eL] = getErrors(traj, theta_virt,x_phys,y_phys);
    
    length_log(:, i + simN * ji) = length;
    eC_log (1, i + simN * ji) = length;
    eC_log (2, i + simN * ji) = eC;
    v_log (1, i + simN * ji) = length;
    v_log (2, i + simN * ji) = Xk(ModelParams.stateindex_vx);

    curvature_current = get_curvature(traj, theta_virt);
    cur_ratio = curvature_current / max_curvature;
    eC_max = trackWidth / 2;
    eC_ratio = (exp(-abs(eC_log(2, i))) - exp(-eC_max)) / (1 - exp(-eC_max));
    v_max = 3;
    v_ratio = v_log(2, i) / v_max;
    punish_v = 0;
    punish_solver = 0;
    if v_log(2, i) < 0.5 && i > 5
        punish_v = -10;
    end
    if info.exitflag ~= 0
        punish_solver = -10;
    end
    Reward_origin = cur_ratio * eC_ratio + (1 - cur_ratio) * v_ratio * 2 + punish_v + punish_solver;
%     reward_step(times, i) = slope * Reward_origin + intercept;
    reward_step(times, i) = Reward_origin;

end

% PlotLog( X_log,U_log,Y,track,track2,simN,Ts)

disp(' ')
figure;
subplot(311)
plot(1:simN, dhat_log(1, :));
hold on
plot(1:simN, disturbance(1, :));

subplot(312)
plot(1:simN, dhat_log(2, :));
hold on
plot(1:simN, disturbance(2, :));

subplot(313)
plot(1:simN, dhat_log(3, :));
hold on
plot(1:simN, disturbance(3, :));

% subplot(312)
% plot(1:simN, dhat_log(2, :));
% 
% subplot(313)
% plot(1:simN, dhat_log(3, :));
% PlotLog( X_log,U_log,Y,track,track2,simN,Ts)
