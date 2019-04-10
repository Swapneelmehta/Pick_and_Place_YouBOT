% Kinematics solver function used to find the next state for every
% iteratioin
 
function [Next_State] = NextState(Current_State,Control_Set,dt,max_wheels,max_joints)

% Constraint vales as per in the model dimensions.
l = 0.235;
w = 0.15;
r = 0.0475;

%Checking the limit of control set and modifying if it exceeds.
for i = 1:prod(size(Control_Set))
    if i>5
        if Control_Set(i) > max_wheels
            Control_Set(i) = max_wheels;
        elseif Control_Set(i) < -max_wheels
            Control_Set(i) = -max_wheels;
        end
    else
        if Control_Set(i) > max_joints
            Control_Set(i) = max_joints;
        elseif Control_Set(i) < -max_joints
            Control_Set(i) = -max_joints;
        end
    end
end

% Calculate motion of joints
change_in_state = Control_Set*dt;
Next_State = zeros(1,12);
for i = 1:prod(size(change_in_state))
    Next_State(3+i) = Current_State(3+i)+change_in_state(i);
end

% Odometry and calculate body twist
V_b = (r/4)*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);1 1 1 1; -1 1 -1 1]*change_in_state(6:9);
if V_b(1) == 0
    dqb = [0;V_b(2);V_b(3)];
else
    dqb = [V_b(1);...
    (V_b(2)*sin(V_b(1))+V_b(3)*(cos(V_b(1))-1))/V_b(1);...
    (V_b(3)*sin(V_b(1))+V_b(2)*(1-cos(V_b(1))))/V_b(1)];
end

%change in chassis configuration
del_q = [1 0 0;0 cos(Current_State(1)) -sin(Current_State(1));...
    0 sin(Current_State(1)) cos(Current_State(1))] * dqb;
for i = 1:numel(del_q)
    Next_State(i) = Current_State(i)+del_q(i);
end
end