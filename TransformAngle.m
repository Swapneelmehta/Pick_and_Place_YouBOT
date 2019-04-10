function [angle] = TransformAngle(frame1,frame2)

% This function solves for obtaining the angle(rad) between two frames

t21 = frame1\frame2;
angle = acos(0.5*(t21(1,1)+t21(2,2)+t21(3,3)-1));
end