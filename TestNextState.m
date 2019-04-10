function [motion] = TestNextState(start,constantControl,dt,totalTime)
motion = zeros(totalTime/dt,13);
motion(1,1:12) = start;
for i = 2:totalTime/dt
    motion(i,1:12)=NextState(motion(i-1,1:12),constantControl,dt,100);
end
csvwrite('testmotion1.csv',motion);
end

