function [ Simulated_Path ] = youBotControl(B_init, t_e_init,t_c_init,...
    t_c_final,T_ce_grasp,T_ce_standoff,K, dt, K_p, K_i, ...
    V_base_max,V_arm_max, traj_W_max,traj_V_max)

    % This function generates a path that simulates control of youBot
    % performing the desired actions.  
    % Provides output to CSVfile.
    
    %screw axis in end-effector frame
    B=transpose([0 0 1 0 0.033 0;0 -1 0 -0.5076 0 0;...
        0 -1 0 -0.3526 0 0;0 -1 0 -0.2176 0 0;...
        0 0 1 0 0 0]);
    
    % Mecanum wheel constants as per given in robot model and dimensions
    l = 0.47/2;
    w = 0.15;
    r = 0.0475;
    F=(r/4)*[0 0 0 0; 0 0 0 0;...
        -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1; -1 1 -1 1;...
        0 0 0 0];
    
    % end effector home position in arm base frame
    M0e = [1 0 0 0.033;0 1 0 0;0 0 1 0.6546;0 0 0 1];
    
    % fixed offset between arm frame in base frame
    tb0 = [1 0 0 0.1662;0 1 0 0;0 0 1 0.0026; 0 0 0 1];
    
    % Generate trajectory
    trajectory = TrajectoryGenerator(t_e_init,t_c_init,...
    t_c_final,T_ce_grasp,T_ce_standoff,K,dt,traj_W_max,traj_V_max);

    % initialize chassis joint values
    joints_current = B_init;
    
    Simulated_Path = zeros(size(trajectory,1)/K-1,13);
    error_list = zeros(size(trajectory,1)/K-1,7);
    for i = 1:(size(trajectory,1)/K)-1
        % Update transforms and Jacobian
        t0e_current = FKinBody(M0e,B,transpose(joints_current(4:8)));
        teb_current = t0e_current\inv(tb0);
        tsb_current = [cos(joints_current(1)) -sin(joints_current(1)) 0 joints_current(2);
            sin(joints_current(1)) cos(joints_current(1)) 0 joints_current(3);
            0 0 1 0.0963;0 0 0 1];
        tse_current = [se3_To_V12(tsb_current*(tb0*t0e_current)) trajectory(i+1,13)];
        J=[Adjoint(teb_current)*F JacobianBody(B,transpose(joints_current(4:8)))];
        
        % Generate controls
        [v,x_err] = FeedbackControl(trajectory(i,1:12),trajectory(i+1,1:12),tse_current,K_p,K_i,dt/K);
        controls = pinv(J)*v;
        
        % Update joint states
        joints_current = [NextState(joints_current,[controls(5:9); controls(1:4)],...
            dt/K,V_base_max,V_arm_max) trajectory(i+1,13)];
        
        % For kth iteration, store xerr and the configuration
        if mod(i,K)==0
            Simulated_Path(i,:) = joints_current;
            error_list(i,:) = [dt*i transpose(x_err)];
        end  
    end
    
    % write csv
    csvwrite('path_1.csv',Simulated_Path);
    csvwrite('x_err.csv',error_list);
    disp('Thus the CSV files are now saved in the desired folder directory');
end