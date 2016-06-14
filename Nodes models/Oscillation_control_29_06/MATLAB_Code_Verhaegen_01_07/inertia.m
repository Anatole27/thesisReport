function [I,xCG] = inertia(dm,l)

% Calculates the CG position and the rotation inertia at the CG
% INPUTS
% dm is a vector of length n with the mass at each node
% l is the length of a beam element
%
% OUTPUTS
% I is the rotation inertia
% xCG is the position of the CG

n = length(dm);
L = (0:l:(n-1)*l)';
xCG = sum(dm.*L)/sum(dm);
I = 0;
for i = 1:n
    I = I + dm(i)*(xCG-(i-1)*l)^2;
end