function trajectory = ScrewTrajectory_modified(Xstart, Xend, Tf, N, method)
% Modified version of screwtrajectory, 
% Returns trajectory list as a 12 vector as output

timegap = Tf / (N - 1);
traj = cell(1, N);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    traj{i} = Xstart * MatrixExp6(MatrixLog6(TransInv(Xstart) * Xend) * s);
end
trajectory = zeros(N,12);

for i=1:N
    trajectory(i,:)=se3_To_V12(traj{i});
end
end

