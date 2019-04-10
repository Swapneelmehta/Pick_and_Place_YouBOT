function [trajectory] = TrajectoryGenerator(t_e_init,t_c_init,...
    t_c_final,T_ce_grasp,T_ce_standoff,K,dt,W_max,V_max)

    min_t = 1;

    %Transforming the initial standoff, initial grasp, final grasp, 
    %final standoff into space frame
    tse_standoff1 = t_c_init * T_ce_standoff;
    tse_grasp1 = t_c_init * T_ce_grasp;
    tse_standoff2 = t_c_final * T_ce_standoff;
    tse_grasp2 = t_c_final * T_ce_grasp;
    
    %calculating the linear distance traveled
    t = ones(1,8);
    t(1)=(norm(t_e_init(1:3,4)-tse_standoff1(1:3,4)))/V_max;
    t(2)=(norm(tse_standoff1(1:3,4)-tse_grasp1(1:3,4)))/V_max;
    t(5)=(norm(tse_standoff1(1:3,4)-tse_standoff2(1:3,4)))/V_max;
    t(6)=(norm(tse_standoff2(1:3,4)-tse_grasp2(1:3,4)))/V_max;
    
    %adding rotation time to angles and round to nearest 0.1s
    t(1)=round(t(1)+TransformAngle(t_e_init,tse_standoff1)/W_max,2);
    t(2)=round(t(2)+TransformAngle(tse_standoff1,tse_grasp1)/W_max,2);
    t(4)=t(2);
    t(5)=round(t(5)+TransformAngle(tse_standoff1,tse_standoff2)/W_max,2);
    t(6)=round(t(6)+TransformAngle(tse_standoff2,tse_grasp2)/W_max,2);
    t(8)=t(6);
    
    n=ones(1,8);
    for i = 1:8
        if t(i) < min_t
            t(i) = min_t;
        end
        n(i) = t(i)/dt*K;
    end
    trajectory = zeros(sum(n),13);
    
    % Transformation matrix to make the trjectory flat
    tse_g1_f = transpose([se3_To_V12(tse_grasp1) 0]);
    tse_g1c_f = transpose([se3_To_V12(tse_grasp1) 1]);
    tse_g2c_f = transpose([se3_To_V12(tse_grasp2) 1]);
    tse_g2_f = transpose([se3_To_V12(tse_grasp2) 0]);
   
    % Generating first section, upto initial standoff
    trajectory(1:n(1),1:12) = ScrewTrajectory_modified(t_e_init,tse_standoff1...
        ,t(1),n(1),3);  
    % Generating initial standoff upto initial grasp
    trajectory(n(1)+1:n(1)+n(2),1:12) = ScrewTrajectory_modified(tse_standoff1,...
        tse_grasp1,t(2),n(2),3);
    % Generating grasp
    trajectory(sum(n(1:2))+1:sum(n(1:3)),1:13) = JointTrajectory(...
        tse_g1_f,tse_g1c_f,t(3),n(3),3);
    % Generating grasp to after grasp standoff
    trajectory(sum(n(1:3))+1:sum(n(1:4)),1:12) = ScrewTrajectory_modified(...
        tse_grasp1,tse_standoff1,t(4),n(4),3);
    % Generating after grasp standoff to final position standoff
    trajectory(sum(n(1:4))+1:sum(n(1:5)),1:12) = ScrewTrajectory_modified(...
        tse_standoff1,tse_standoff2,t(5),n(5),3);
    % Generating final position standoff to final position grasp
    trajectory(sum(n(1:5))+1:sum(n(1:6)),1:12) = ScrewTrajectory_modified(...
        tse_standoff2,tse_grasp2,t(6),n(6),3);
    % Adding gripper closed state
    trajectory(sum(n(1:3))+1:sum(n(1:6)),13) = 1;
    %Then generating the release of gripper
    trajectory(sum(n(1:6))+1:sum(n(1:7)),1:13) = JointTrajectory(...
        tse_g2c_f,tse_g2_f,t(7),n(7),3);
    % Genenerating final position of standoff
    trajectory(sum(n(1:7))+1:sum(n(1:8)),1:12) = ScrewTrajectory_modified(...
        tse_grasp2,tse_standoff2,t(8),n(8),3);
    trajectory(sum(n(1:7))+1:sum(n(1:8)),13) = 0;
    % Writing files in csv form
    csvwrite('traj.csv',trajectory);

end

