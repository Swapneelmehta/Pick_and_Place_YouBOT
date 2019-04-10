%Input as a 12-vector configuration and output as 4x4 se3 transform

function [ transform ] = V12_To_se3( config )
transform = zeros(4);
transform(1,1:3) = config(1:3);
transform(2,1:3) = config(4:6);
transform(3,1:3) = config(7:9);
transform(1:3,4) = transpose(config(10:12));
transform(4,4) = 1;
end