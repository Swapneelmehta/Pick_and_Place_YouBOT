function [flat] = se3_To_V12(transform)
%This function takes in transform in standard 4 x 4 matrix
% and returns 12 vector as output for mathematical purposes
flat = [transform(1,1:3) transform(2,1:3) transform(3,1:3) ...
    transpose(transform(1:3,4))];
end