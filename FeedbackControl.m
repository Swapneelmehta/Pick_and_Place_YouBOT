function [ v,x_err ] = FeedbackControl( xd,xd_next,tse,K_p,K_i,dt )

int_steps = 30; 
% Number of past timesteps to accumulate integral error

% Initializing the variable i_error if it does not exist already
if ~exist('i_error','var')
    persistent i_error;
    i_error = zeros(6,int_steps);
end

% Transforming into 4x4 matrix
tse_updated = V12_To_se3(tse);
xd_ = V12_To_se3(xd);
xd_next_ = V12_To_se3(xd_next);

% Updating the PI gains in system
x_err = se3ToVec(MatrixLog6(tse_updated\xd_));
i_err_i = x_err * dt;
i_error = [i_err_i i_error(:,1:int_steps-1)];

% Output calculated twists 
vd = se3ToVec(MatrixLog6(xd_\xd_next_))*(1/dt);
v = Adjoint(tse_updated\xd_)*vd + K_p * x_err + K_i*sum(i_error,2);

end