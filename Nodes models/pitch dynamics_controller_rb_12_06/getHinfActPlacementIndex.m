function [Sigma,corr] = getHinfActIndices(w_act,w_mode,B,C,om,z)

% Author : Anatole VERHAEGEN
% 
% This function yields a matrix Sigma which is the Hinf actuator placement
% index matrix
%
% Outputs
% Sigma(i,j) is the placement index for the ith mode and jth actuator
% corr is the correlation matrix between actuators
%
% Inputs
% w_act,w_mode : weights for each actuator and mode
% B,C : Input and output matrices of modal SS form 2
% om, z: modal natural fq and damping

n = length(w_mode);
p = length(w_act);

% Gij : norm of ith mode and jth actuator :
Gij = zeros(n,p);
for i = 1:n
    for j = 1:p
        Gij(i,j) = getHinfNorm_mode_actuator(B,C,i,j,om,z);
    end
end

% Norm of ith mode :
Gi = zeros(n,1);
for i = 1:n
    Gi(i) = norm(Gij(i,:));
end

% Vector of the squares of modal norms
gij = Gij.^2;

% Correlation matrix
corr = zeros(p,p);
for i = 1:p
    for j = 1:i
        gi = gij(:,i);
        gk = gij(:,j);
        corr(i,j) = gi'*gk/(norm(gi)*norm(gk));
        corr(j,i) = corr(i,j);
    end
end

% Norm of system :
G = max(Gi);

% Index matrix
Sigma = diag(w_mode)*Gij*diag(w_act)/G;
