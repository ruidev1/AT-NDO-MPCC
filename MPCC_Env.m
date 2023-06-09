classdef MPCC_Env < rl.env.MATLABEnvironment
    %MPCC_ENV: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
        isTrain = true;
        % Specify and initialize environment's necessary properties    
        CarModel = 'ORCA'

        MPC_vars
        ModelParams

        % Simulation lenght and plotting
%         simN = 20
        simN = 1
        steps = 400;
        stepIndex = 0;
        N = 40
        %0=no plots, 1=plot predictions
        plotOn = 0
        %0=real time iteration, 1=fixed number of QP iterations, 2=fixed number of damped QP iterations
        QP_iter = 2
        % number of cars 
        % there are two examples one with no other cars and one with 4 other cars
        % (inspired by the set up shown in the paper)
        n_cars = 1; % no other car
%         n_cars = 5 % 4 other cars
        Y

        % times of simulation
        sims = 0;

        % track, traj and border
        track
        track2
        trackWidth
        traj
        borders
        tl
        TrackMPC

        % log
        reward_log
        X_log
        U_log
        B_log
        Params_log
        filename = 'Info_log.mat'
        m
        qpTime_log
        length_log = zeros(1, 400);
        % total simN steps
        total_count = 0
        cycle_flag = 0
        % used to further cauculate the mean and max value along one step
        obs_log = zeros(2, 20)
        eC_v_log = zeros(2, 400)
        curvature_log = zeros(2, 20)
        curvature_pred_log = zeros(2, 40)
        curvature_k_log = zeros(1, 20)
        curvature_k_pred_log = zeros(1, 40)

        last_closestIdx

        max_curvature = 0

        disturbance = zeros(3, 400)
        z = zeros(3, 1);
        dhat_test = zeros(3, 1)
        
    end
    
    properties
        % Initialize system state [x,y,phi,vx,vy,omega,theta]'
        State = zeros(7,41)
        x0 = zeros(7,1)
        x = zeros(7,41)
        u = zeros(3,40)
        uprev = zeros(3,1)
        b
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = MPCC_Env()
            % Initialize Observation settings
%             ObservationInfo = rlNumericSpec([1 1]);
            ObservationInfo = rlNumericSpec([6 1]);
            ObservationInfo.Name = 'Vehicle States';
%             ObservationInfo.Description = 'x_phy, y_phy, x_virt, y_virt, eC, eL';
%             ObservationInfo.Description = 'eC_error_mean, eC_error_max, driving_length, curvature_x, curvature_y';
%             ObservationInfo.Description = 'eC_error_mean, eC_error_max, v_mean, v_max, curvature_x, curvature_y';
%             ObservationInfo.Description = 'eC_error_mean, eC_error_max, driving_length';
%             ObservationInfo.Description = ['eC_error_mean, eC_error_max, v_mean, v_max, pred_eC_mean,' ...
%                 'pred_eC_max, pred_v_mean, pred_v_max, curvature_mean, curvature_mean_pred'];
%             ObservationInfo.Description = ['eC_error_mean, eC_error_max, v_mean, v_max, pred_eC_mean,' ...
%                 'pred_eC_max, pred_v_mean, pred_v_max, curvature_x, curvature_y'];
            ObservationInfo.Description = 'curvature';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([4 1]);
%             ActionInfo = rlNumericSpec([2 1]);
%             ActionInfo.Name = 'Cost Weights';
%             ActionInfo.Name = 'qC, qL, qOmega, rD, rDelta rVtheta';
%             ActionInfo.Name = 'qC, qL, qOmega';
%             ActionInfo.Name = 'qC, qL, qOmega, rVtheta';
%             ActionInfo.Description = 'qC';
            ActionInfo.Name = 'qC, qVtheta, qL, qOmega';
%             ActionInfo.LowerLimit = [1e-5; 100; 1e-6; 0.0002];
%             ActionInfo.UpperLimit = [100; 10000; 1e-4; 20];
%             ActionInfo.LowerLimit = [0.01; 0.01];
%             ActionInfo.UpperLimit = [10; 1];
            ActionInfo.LowerLimit = [0.01; 0.01; 500; 1e-6];
            ActionInfo.UpperLimit = [10; 1; 1500; 1e-4];
%             ActionInfo.LowerLimit = [0.01];
%             ActionInfo.UpperLimit = [200];
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);

            % Load Parameters
            this.MPC_vars = getMPC_vars(this.CarModel);
            this.ModelParams = getModelParams(this.MPC_vars.ModelNo);
            % choose optimization interface options: 'Yalmip','CVX','hpipm','quadprog'
            this.MPC_vars.interface = 'hpipm';
            
            % import an plot track
            load MPCC_Solver/MPCC/Tracks/track2.mat track2
            % shrink track by half of the car widht plus safety margin
            % TODO implement orientation depending shrinking in the MPC constraints
            safteyScaling = 1.5;
            [this.track,this.track2] = borderAdjustment(track2,this.ModelParams,safteyScaling);
            
            this.trackWidth = norm(this.track.inner(:,1)-this.track.outer(:,1));
            % plot shrinked and not shrinked track 
%             figure(1);
%             plot(this.track.outer(1,:),this.track.outer(2,:),'r')
%             hold on
%             plot(this.track.inner(1,:),this.track.inner(2,:),'r')
%             plot(this.track2.outer(1,:),this.track2.outer(2,:),'k')
%             plot(this.track2.inner(1,:),this.track2.inner(2,:),'k')

            % Fit spline to track
            % TODO spline function only works with regular spaced points.
            % Fix add function which given any center line and bound generates equlally
            % space tracks.
            [this.traj, this.borders] =splinify(this.track);
            this.tl = this.traj.ppy.breaks(end);

            curvatures = zeros(1, 500);
            for t = 1: 500
                theta = t * this.tl / 500;
                curvatures(t) = get_curvature(this.traj, theta);
            end
            this.max_curvature = max(curvatures);

            
            
            % store all data in one struct
            this.TrackMPC = struct('traj',this.traj,'borders',this.borders,'track_center',this.track.center,'tl',this.tl);

            % Initialize logging arrays
            this.X_log = zeros(this.ModelParams.nx*(this.MPC_vars.N+1),this.steps);
            this.U_log = zeros(3*this.MPC_vars.N,this.steps);
            this.B_log = zeros(4*this.MPC_vars.N,this.steps);
            this.qpTime_log = zeros(1,this.steps);
            this.reward_log = zeros(1, this.steps);
            this.Params_log = zeros(4, this.steps);

            this.m = matfile(this.filename, 'Writable', true); %Note: writable is true by default IF the file does not exist
            
            % Initialize property values and pre-compute necessary values
%             updateActionInfo(this);
        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            this.stepIndex = this.stepIndex + 1;
%             [qC; qL; qOmega; rD; rDelta; rVtheta] = Action;
            this.sims = this.sims + 1;
%             this.MPC_vars.qC = Action;
            this.MPC_vars.qC = Action(1);
%             this.MPC_vars.qVtheta= 1;
            this.MPC_vars.qVtheta = Action(2);
            this.MPC_vars.qL = Action(3);
            this.MPC_vars.qOmega = Action(4);
%             this.MPC_vars.rD = Action(4);
%             this.MPC_vars.rDelta = Action(5);
%             this.MPC_vars.rVtheta = Action(4);
%             PlotAction(Action, this.MPC_vars, this.simN)
            if this.plotOn == 1
                n = 3;
                PlotParameters(this.MPC_vars, this.simN, n, this.sims);
            end
            
            % disturbance
            decay = 0.001;
            if this.sims > 1
                this.disturbance(:, this.sims) = this.disturbance(:, this.sims-1) - decay * this.disturbance(:, this.sims-1) + 0.001 * wgn(3,1,0);
%                 this.disturbance(:, this.sims) = this.disturbance(:, this.sims-1) - decay * this.disturbance(:, this.sims-1);
            end
            d = this.disturbance(:, this.sims);
%             d = 0;
            
            % Simulation
            for i = 1: this.simN
                this.total_count = this.total_count + 1;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%% MPCC-Call %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                % augment state and inputs by shifting previus optimal solution
                [this.x,this.u] = augState(this.x,this.u,this.x0,this.MPC_vars,this.ModelParams,this.tl, d);
                %  formulate MPCC problem and solve it
                if this.QP_iter == 0
                    [this.x, this.u, this.b, ~,info] = optimizer_mpcc(this.TrackMPC,this.MPC_vars,this.ModelParams, this.n_cars, this.Y, this.x, this.u, this.x0, this.uprev);
                    this.qpTime_log(i) = info.QPtime;
                elseif this.QP_iter == 1
                    % doing multiple "SQP" steps
                    for k = 1:2
                        [this.x, this.u, this.b, ~,info] = optimizer_mpcc(this.TrackMPC,this.MPC_vars,this.ModelParams, this.n_cars, this.Y, this.x, this.u, this.x0, this.uprev);
                        this.qpTime_log(i) = this.qpTime_log(i) + info.QPtime;
                    end
                elseif this.QP_iter == 2
                    % doing multiple damped "SQP" steps
                    for k = 1:2
                        Iter_damping = 0.75; % 0 no damping
                        [x_up, u_up, this.b, ~,info] = optimizer_mpcc(this.TrackMPC,this.MPC_vars,this.ModelParams, this.n_cars, this.Y, this.x, this.u, this.x0, this.uprev, this.dhat_test);
                        this.x = Iter_damping*this.x + (1-Iter_damping)*x_up;
                        this.u = Iter_damping*this.u + (1-Iter_damping)*u_up;
                        this.qpTime_log(i) = this.qpTime_log(i) + info.QPtime;
                    end
                else
                    error('invalid QP_iter value')
                end

                % NDO
                xdot = fx_bicycle(0, this.x(:,1), this.u(:, 1), this.ModelParams);
   
                phi = this.x0(3);
                omega = this.x0(6);
            
                R = [cos(phi), -sin(phi), 0;
                     sin(phi), cos(phi), 0;
                     0, 0, 1];
                R_dot = [-sin(phi) * omega, -cos(phi) * omega, 0;
                          cos(phi) * omega, -sin(phi) * omega, 0;
                          0, 0, 0];

%                 L = diag([1.8, 0.5, 0.25]);
                L = diag([1.2, 0.5, 0.2]);
               
                x2 = this.x(4:6, 1);
                f2x=xdot(4:6);
%                 x2_dot = fx_bicycle_disturbance(0, this.x(:, 1), this.u(:, 1), this.ModelParams, d);
                px = L * x2;
                this.dhat_test = this.z + px;
%                 dhat_log(:, i) = dhat_test;
%                 if i == 1
%                     dhat = z + px;
%                 end
%                 dhat_dot = -L * dhat + L * (x2_dot(4:6) - f2x);
%                 z_dot = -L *this.z - L *(f2x + px);
%                 px_dot = L * x2_dot(4:6);
%                 z_dot_test = dhat_dot - px_dot;
%                 dhat_dot_test = z_dot + px_dot;
            %     dhat = dhat + dhat_dot * Ts;
%                 [~,inid]=ode45(@(t,dhat) -L * dhat + L * (x2_dot(6) - f2x),[0 Ts],dhat);
%                 dhat=inid(end,:)';
%                 z_direct = z + z_dot * Ts;
                [~,iniz]=ode45(@(t,z) -L *z - L *(f2x + px),[0 this.MPC_vars.Ts],this.z);
                this.z=iniz(end,:)';
            %     x2_test = x2 + x2_dot(6) * Ts;
%                 [~,inix]=ode45(@(t,x) fx_bicycle_disturbance(0, this.x, this.u(:, 1), this.ModelParams, d),[0 Ts],this.x(:, 1));
%                 x2_test = inix(end, 4:6)';
            
%                 px_appro = L * x2 + L * x2_dot(4:6) * Ts;
%                 px_fine_intergral = L * x2_test;
%             
%                 z_should = dhat - px_appro;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%% simulate system %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                this.x0 = SimTimeStep(this.x(:,1),this.u(:,1),this.MPC_vars.Ts,this.ModelParams, d)';
                this.x0 = unWrapX0(this.x0);
                [ theta, this.last_closestIdx] = findTheta(this.x0,this.track.center,this.traj.ppx.breaks,this.trackWidth,this.last_closestIdx);
                this.x0(this.ModelParams.stateindex_theta) = theta;
                this.uprev = this.u(:,1);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%% plotting and logging %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
                if this.plotOn == 1
                    PlotPrediction(this.x,this.u,this.b,this.Y,this.track,this.track2,this.traj,this.MPC_vars,this.ModelParams)
                end
                
                % log predictions and time
                this.X_log(:,this.sims) = reshape(this.x,(this.N+1)*7,1);
                this.U_log(:,this.sims) = reshape(this.u,(this.N)*3,1);
                this.B_log(:,this.sims) = reshape(this.b,this.N*4,1);
                this.Params_log(1, this.sims) = Action(1);
                this.Params_log(2, this.sims) = Action(2);
                this.Params_log(3, this.sims) = Action(3);
                this.Params_log(4, this.sims) = Action(4);

                % store the necesseary info in each sampling step used for later observation calculation
                Xk = this.x(:,1);
                Xk_next = this.x0;
%                 Xk = this.x0;
                x_phys = Xk(1);
                y_phys = Xk(2);
                theta_virt=mod(Xk(end),this.traj.ppx.breaks(end));
                x_phys_next = Xk_next(1);
                y_phys_next = Xk_next(2);
                theta_virt_next=mod(Xk_next(end),this.traj.ppx.breaks(end));
%                 x_virt=ppval(this.traj.ppx,theta_virt);
%                 y_virt=ppval(this.traj.ppy,theta_virt);
                [eC, eL] = this.getErrors(this.traj, theta_virt_next,x_phys_next,y_phys_next);
                if i == 1
                    first_theta = mod(Xk(end),this.traj.ppx.breaks(end));
                end

                if i == this.simN
                    last_theta = mod(Xk(end),this.traj.ppx.breaks(end));
                end
                this.obs_log(1, i) = eC;
                this.obs_log(2, i) = Xk_next(this.ModelParams.stateindex_vx);
                this.eC_v_log(1, this.sims) = eC;
                this.eC_v_log(2, this.sims) = Xk_next(this.ModelParams.stateindex_vx);
%                 this.curvature_log(1, i) = ppval(this.traj.ddppx,theta_virt);
%                 this.curvature_log(2, i) = ppval(this.traj.ddppy,theta_virt);
                this.curvature_k_log(1, i) = get_curvature(this.traj, theta_virt);

                if this.isTrain == false
                    length = Xk(end);
                    if this.total_count > 1 && length < this.length_log(1, this.total_count-1)
                        this.cycle_flag = 1;
                    end
                    if this.cycle_flag == 1
                        length = length + this.tl;
                    end
                    this.length_log(1, this.total_count) = length;
                    this.m.out(1, this.total_count) = length;
                    this.m.out(2, this.total_count) = eC;
                    this.m.out(3, this.total_count) = length;
                    this.m.out(4, this.total_count) = Xk(this.ModelParams.stateindex_vx);
                end

            end
            
%             i = this.MPC_vars.N+1; % observation from N
%             Xk = this.x(:,1);
%             x_phys = Xk(1);
%             y_phys = Xk(2);
%             theta_virt=mod(Xk(end),this.traj.ppx.breaks(end));
%             x_virt=ppval(this.traj.ppx,theta_virt);
%             y_virt=ppval(this.traj.ppy,theta_virt);
%             [eC, eL] = this.getErrors(this.traj, theta_virt,x_phys,y_phys);
% 
%             Observation = [x_phys;y_phys;x_virt;y_virt;eC;eL];
%             this.State = this.x;
            eC_pred_log = zeros(1, this.MPC_vars.N);
            v_pred_log = zeros(1, this.MPC_vars.N);
            for j = 1: this.MPC_vars.N
                Xj = this.x(:, j);
                x_phys_pred = Xj(1);
                y_phys_pred = Xj(2);
                theta_virt_pred=mod(Xj(end),this.traj.ppx.breaks(end));
                [eC_pred, eL_pred] = this.getErrors(this.traj, theta_virt_pred,x_phys_pred,y_phys_pred);
                eC_pred_log(1, j) = eL_pred;
                v_pred_log(1, j) = Xj(this.ModelParams.stateindex_vx);
%                 this.curvature_pred_log(1, j) = ppval(this.traj.ddppx,theta_virt_pred);
%                 this.curvature_pred_log(2, j) = ppval(this.traj.ddppy,theta_virt_pred);
                dphi = ppval(this.traj.dppx,theta_virt_pred);
                ddphi = ppval(this.traj.ddppx,theta_virt_pred);
                domega = ppval(this.traj.dppy,theta_virt_pred);
                ddomega = ppval(this.traj.ddppy,theta_virt_pred);
                numerator = abs(dphi * ddomega - domega * ddphi);
                denominator = sqrt((dphi ^ 2 + domega ^2) ^ 3);
                this.curvature_k_pred_log(1, j) = numerator / denominator;
            end
            eC_pred_log = abs(eC_pred_log);
            v_pred_log = abs(v_pred_log);
            this.curvature_pred_log = abs(this.curvature_pred_log);

            this.obs_log = abs(this.obs_log);
            this.curvature_log = abs(this.curvature_log);
            if last_theta < first_theta
                driving_length = (last_theta + this.tl - first_theta) / this.tl;
            else
                driving_length = (last_theta - first_theta) / this.tl;
            end
            curvature_x = ppval(this.traj.ddppx,theta_virt);
            curvature_y = ppval(this.traj.ddppy,theta_virt);
            curvature_current = get_curvature(this.traj, theta_virt);
%             Observation = [mean(this.obs_log(1,:)); max(this.obs_log(1,:)); mean(this.obs_log(2,:)); max(this.obs_log(2,:)); mean(eC_pred_log); max(eC_pred_log); mean(v_pred_log); max(v_pred_log); mean(this.curvature_k_log(1,:)); mean(this.curvature_k_pred_log(1,:))];
%             Observation = [mean(this.obs_log(1,:)); max(this.obs_log(1,:)); mean(this.obs_log(2,:)); max(this.obs_log(2,:)); mean(eC_pred_log); max(eC_pred_log); mean(v_pred_log); max(v_pred_log); curvature_x; curvature_y];
%             Observation = [mean(this.obs_log); max(this.obs_log); driving_length];
            x_virt=ppval(this.traj.ppx,theta_virt);
            y_virt=ppval(this.traj.ppy,theta_virt);
            Observation = [curvature_current; x_phys; y_phys; theta_virt; eC; this.obs_log(2, 1)];
%             Observation = [curvature_current];
            this.State = this.x;

            LoggedSignals = [];
            
            
            % Check terminal condition
%             IsDone = (this.sims > 5 && this.obs_log(2, 1) < 0.5) || info.exitflag ~= 0;
            IsDone = false;
            this.IsDone = IsDone;
            
            % Get reward
%             Reward = getReward(this, this.obs_log, driving_length);
%             Reward = getReward(this, this.obs_log, this.curvature_k_pred_log);
            Reward = getReward(this, curvature_current, info.exitflag);
            this.reward_log(1, this.stepIndex) = Reward;
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
            if ishandle(4)
                close(4)
            end
            this.sims = 0;
            this.stepIndex = 0;
           
            %% Disturbance
%             this.disturbance(1, 1:this.steps) = 0.01;
%             this.disturbance(2, 1:this.steps) = 0.5;
%             this.disturbance(3, 1:this.steps) = 1;
            this.disturbance(1, 1:this.steps) = 0.2;
            this.disturbance(2, 1:this.steps) = 0.6;
            this.disturbance(3, 1:this.steps) = 0.5;
            %% Define starting position
%             startIdx = 40; %point (in terms of track centerline array) allong the track 
            startIdx = 1;
%             startIdx = randi([1 660]);
            % where the car starts, on the center line, aligned with the track, driving
            % straight with vx0
            %since the used bicycle model is not well defined for slow velocities use vx0 > 0.5
            this.N = this.MPC_vars.N;
            Ts = this.MPC_vars.Ts;
            if this.CarModel == "ORCA"
%                 vx0 = 1;
                vx0 = 0;
            elseif this.CarModel == "FullSize"
                vx0 = 15;
            end
            
            % find theta that coresponds to the 10th point on the centerline
            [theta, ~] = findTheta([this.track.center(1,startIdx),this.track.center(2,startIdx)],this.track.center,this.traj.ppx.breaks,this.trackWidth,startIdx);
            
            this.x0 = [this.track.center(1,startIdx),this.track.center(2,startIdx),... % point on centerline
                  atan2(ppval(this.traj.dppy,theta),ppval(this.traj.dppx,theta)),... % aligned with centerline
                  vx0 ,0,0,theta]'; %driving straight with vx0, and correct theta progress

            % the find theta function performs a local search to find the projection of
            % the position onto the centerline, therefore, we use the start index as an
            % starting point for this local search
            this.last_closestIdx = startIdx;

            % First initial guess
            this.x = repmat(this.x0,1,this.N+1); % all points identical to current measurment
            % first inital guess, all points on centerline aligned with centerline
            % spaced as the car would drive with vx0
            for i = 2:this.N+1
                theta_next = this.x(this.ModelParams.stateindex_theta,i-1)+Ts*vx0;
                phi_next = atan2(ppval(this.traj.dppy,theta_next),ppval(this.traj.dppx,theta_next));
                % phi_next can jump by two pi, make sure there are no jumps in the
                % initial guess
                if (this.x(this.ModelParams.stateindex_phi,i-1)-phi_next) < -pi
                    phi_next = phi_next-2*pi;
                end
                if (this.x(this.ModelParams.stateindex_phi,i-1)-phi_next) > pi
                    phi_next = phi_next+2*pi;
                end
                this.x(:,i) = [ppval(this.traj.ppx,theta_next),ppval(this.traj.ppy,theta_next),... % point on centerline
                          phi_next,... % aligned with centerline
                          vx0 ,0,0,theta_next]'; %driving straight with vx0, and correct theta progress
            end
            
            this.u = zeros(3,this.N); % zero inputs
            this.uprev = zeros(3,1); % last input is zero

            
            % Ohter cars
            this.Y = ObstacelsState(this.track,this.traj,this.trackWidth,this.n_cars);
            
            if size(this.Y,2) ~= this.n_cars-1
                error('n_cars and the number of obstacles in "Y" does not match')
            end

            

            % initializtion
            % solve problem 5 times without applying input
            % inspiered by sequential quadratic programming (SQP)
            for i = 1:5
                % formulate MPCC problem and solve it
                Iter_damping = 0.5; % 0 no damping
                [x_up, u_up, this.b, ~,~] = optimizer_mpcc(this.TrackMPC,this.MPC_vars,this.ModelParams, this.n_cars, this.Y, this.x, this.u, this.x0, this.uprev, this.dhat_test);
                this.x = Iter_damping*this.x + (1-Iter_damping)*x_up;
                this.u = Iter_damping*this.u + (1-Iter_damping)*u_up;
            
                if this.plotOn == 1
                    % plot predictions
                    PlotPrediction(this.x,this.u,this.b,this.Y,this.track,this.track2,this.traj,this.MPC_vars,this.ModelParams)
                end
            end
            
            % TODO: sum all steps
            % TODO: experience choose the observation from x0 or xN
            i = this.MPC_vars.N+1; % observation from N
            Xk = this.x(:,1); % observation from 1 or N
            x_phys = Xk(1);
            y_phys = Xk(2);
            theta_virt=mod(Xk(end),this.traj.ppx.breaks(end));
            [eC, eL] = this.getErrors(this.traj, theta_virt,x_phys,y_phys);
            eC_pred_log = zeros(1, this.MPC_vars.N);
            v_pred_log = zeros(1, this.MPC_vars.N);
            for j = 1: this.MPC_vars.N
                Xj = this.x(:, j);
                x_phys_pred = Xj(1);
                y_phys_pred = Xj(2);
                theta_virt_pred=mod(Xj(end),this.traj.ppx.breaks(end));
                [eC_pred, eL_pred] = this.getErrors(this.traj, theta_virt_pred,x_phys_pred,y_phys_pred);
                eC_pred_log(1, j) = eL_pred;
                v_pred_log(1, j) = Xj(this.ModelParams.stateindex_vx);
%                 this.curvature_pred_log(1, j) = ppval(this.traj.ddppx,theta_virt_pred);
%                 this.curvature_pred_log(2, j) = ppval(this.traj.ddppy,theta_virt_pred);
                dphi = ppval(this.traj.dppx,theta_virt_pred);
                ddphi = ppval(this.traj.ddppx,theta_virt_pred);
                domega = ppval(this.traj.dppy,theta_virt_pred);
                ddomega = ppval(this.traj.ddppy,theta_virt_pred);
                numerator = abs(dphi * ddomega - domega * ddphi);
                denominator = sqrt((dphi ^ 2 + domega ^2) ^ 3);
                this.curvature_k_pred_log(1, j) = numerator / denominator;
            end
            eC_pred_log = abs(eC_pred_log);
            v_pred_log = abs(v_pred_log);
            this.curvature_pred_log = abs(this.curvature_pred_log);


%             InitialObservation = [x_phys;y_phys;x_virt;y_virt;eC;eL];
            curvature_x = ppval(this.traj.ddppx,theta_virt);
            curvature_y = ppval(this.traj.ddppy,theta_virt);
            dphi = ppval(this.traj.dppx,theta_virt);
            ddphi = ppval(this.traj.ddppx,theta_virt);
            domega = ppval(this.traj.dppy,theta_virt);
            ddomega = ppval(this.traj.ddppy,theta_virt);
            numerator = abs(dphi * ddomega - domega * ddphi);
            denominator = sqrt((dphi ^ 2 + domega ^2) ^ 3);
            curvature_k = numerator / denominator;

            curvature_current = get_curvature(this.traj, theta_virt);

%             InitialObservation = [eC; eC; 0; 0; mean(eC_pred_log); max(eC_pred_log); mean(v_pred_log); max(v_pred_log); curvature_k; mean(this.curvature_k_pred_log(1,:))];
%             InitialObservation = [eC; eC; 0; 0; mean(eC_pred_log); max(eC_pred_log); mean(v_pred_log); max(v_pred_log); curvature_x; curvature_y];
%             InitialObservation = [eC; eC; 0];
            x_virt=ppval(this.traj.ppx,theta_virt);
            y_virt=ppval(this.traj.ppy,theta_virt);
            InitialObservation = [curvature_current; x_phys; y_phys; theta_virt; eC; 0];
%             InitialObservation = [curvature_current];
            this.State = this.x;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        function [eC, eL] = getErrors(~, pathinfo, theta_virt,x_phys,y_phys)
            dxdth=ppval(pathinfo.dppx,theta_virt); % d x / d theta
            dydth=ppval(pathinfo.dppy,theta_virt); % d y / d theta
        
            % virtual positions
            x_virt=ppval(pathinfo.ppx,theta_virt);
            y_virt=ppval(pathinfo.ppy,theta_virt);
            
            phi_virt=atan2(dydth,dxdth);
            
            % define these to reduce calls to trig functions
            sin_phi_virt = sin(phi_virt);
            cos_phi_virt = cos(phi_virt);
        
            % contouring and lag error estimates
            eC = -sin_phi_virt*(x_virt - x_phys) + cos_phi_virt*(y_virt - y_phys);
            eL =  cos_phi_virt*(x_virt - x_phys) + sin_phi_virt*(y_virt - y_phys);
           
        end

        % Discrete force 1 or 2
        function force = getForce(this,action)
            if ~ismember(action,this.ActionInfo.Elements)
                error('Action must be %g for going left and %g for going right.',-this.MaxForce,this.MaxForce);
            end
            force = action;           
        end
        % update the action info based on max force
        function updateActionInfo(this)
            this.ActionInfo.Elements = this.MaxForce*[-1 1];
        end
        
        % Reward function
        function Reward = getReward(this, curvature_current, exitflag)
%             Xk = this.x(:,1); % observation from 1
%             x_phys = Xk(1);
%             y_phys = Xk(2);
%             theta_virt=mod(Xk(end),this.traj.ppx.breaks(end));
% %             x_virt=ppval(this.traj.ppx,theta_virt);
% %             y_virt=ppval(this.traj.ppy,theta_virt);
%             [eC, eL] = this.getErrors(this.traj, theta_virt,x_phys,y_phys);
% %             Reward = exp(-((eC * eC) / 0.001 + (eL * eL) / 0.00001));
            
%             Reward = driving_length * 25;
%             mean_eC = mean(obs_log(1,:));
%             shifted_length = driving_length * 25 - 1; % [0,1] to [-1, 0]
%             error_reward = exp(-(mean_eC * mean_eC) / 0.001);
%             length_reward = exp(shifted_length);s
%             Reward = length_reward;
%             Reward = 0.5 * error_reward + 0.5 * length_reward;
%             Reward = 10* Reward;
            
%             mean_v = mean(obs_log(2,:));
%             mean_curvature_k_pred = mean(curvature_k_pred_log);
%             if mean_curvature_k_pred > 2
%                 Reward =  2 * exp(-(mean_eC * mean_eC) / 0.001) + mean_v;
%             else
%                 Reward = mean_v;
%             end
%             Reward = mean_v - mean_eC;

            cur_ratio = curvature_current / this.max_curvature;
            eC_max = this.trackWidth / 2;
            eC_ratio = (exp(-this.obs_log(1,1)) - exp(-eC_max)) / (1 - exp(-eC_max));
            v_max = 3;
            v_ratio = this.obs_log(2,1) / v_max;
            punish_v = 0;
            punish_solver = 0;
            if this.obs_log(2,1) < 0.5 && this.sims > 5
                punish_v = -10;
            end
            if exitflag ~= 0
                punish_solver = -10;
            end
            Reward_origin = cur_ratio * eC_ratio + (1 - cur_ratio) * v_ratio * 2 + punish_v + punish_solver;
            Reward = Reward_origin;
%             Reward = eC_ratio + punish_v;
            slope = 10;
            intercept = -3;
%             Reward = slope * eC_ratio + intercept;
%             Reward = slope * Reward_origin;
        
        end
        
        % (optional) Visualization method
        function plot(this)
            % Initiate the visualization
            
            % Update the visualization
            envUpdatedCallback(this)
        end
        
%         % (optional) Properties validation through set methods
%         function set.State(this,state)
%             validateattributes(state,{'numeric'},{'finite','real','vector','numel',4},'','State');
%             this.State = double(state(:));
%             notifyEnvUpdated(this);
%         end
%         function set.HalfPoleLength(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','HalfPoleLength');
%             this.HalfPoleLength = val;
%             notifyEnvUpdated(this);
%         end
%         function set.Gravity(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Gravity');
%             this.Gravity = val;
%         end
%         function set.CartMass(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','CartMass');
%             this.CartMass = val;
%         end
%         function set.PoleMass(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','PoleMass');
%             this.PoleMass = val;
%         end
%         function set.MaxForce(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','MaxForce');
%             this.MaxForce = val;
%             updateActionInfo(this);
%         end
%         function set.Ts(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','Ts');
%             this.Ts = val;
%         end
%         function set.AngleThreshold(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','AngleThreshold');
%             this.AngleThreshold = val;
%         end
%         function set.DisplacementThreshold(this,val)
%             validateattributes(val,{'numeric'},{'finite','real','positive','scalar'},'','DisplacementThreshold');
%             this.DisplacementThreshold = val;
%         end
%         function set.RewardForNotFalling(this,val)
%             validateattributes(val,{'numeric'},{'real','finite','scalar'},'','RewardForNotFalling');
%             this.RewardForNotFalling = val;
%         end
%         function set.PenaltyForFalling(this,val)
%             validateattributes(val,{'numeric'},{'real','finite','scalar'},'','PenaltyForFalling');
%             this.PenaltyForFalling = val;
%         end
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
        end
    end
end
