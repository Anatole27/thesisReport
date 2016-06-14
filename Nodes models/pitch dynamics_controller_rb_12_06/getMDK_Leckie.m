function [M,D,K,Bo,Coq,Cor,Coa,Doq,Dor,Doa] = getMDK_Leckie(EI,l,n,dm,Diam)

% Author : Anatole VERHAEGEN
% This function computes M,D,K matrices of the second-order structural
% model plus input and output matrices
%
% Inputs :
% EI : stiffness along the beam, (n-1 x 1)
% qSCLa : lift derivative wrt AoA. (n-1 x 1)
% l : length of an element of beam
% Diam : diameter along the structure (n x 1)

% Outputs
% M : mass matrix (n x n)
% D : damping matrix (n x n)
% K : stiffness matrix (n x n)
% Bo : input matrix (n x p)
% Coq : output displacement matrix (r_q x n)
% Cor : output velocity/rate matrix (r_v x n)

% Stiffness matrix

K11 = zeros(n); K22 = K11; K12 = K11;

for i = 1:n-1
    k11 = 12*EI(i)/l^3*[1 -1; -1 1];
    k12 = 6*EI(i)/l^2*[1 1; -1 -1];
    k22 = EI(i)/l*[4 2; 2 4];
    
    K11(i:i+1,i:i+1) = K11(i:i+1,i:i+1) + k11;
    K12(i:i+1,i:i+1) = K12(i:i+1,i:i+1) + k12;
    K22(i:i+1,i:i+1) = K22(i:i+1,i:i+1) + k22;
end

K21 = K12';

K = K11-K12*inv(K22)*K21; % Hyp : no inertia, no local torque
K = (K+K')/2;

% Damping matrix
D = 1/13000*K;

% Mass matrix
M = diag(dm);

% Input
Bo = eye(n);

% Stress measurement :
angle2stress = zeros(n-2,n);
angle2stress(1:n-2,1:n-2) = -1/(2*l)*eye(n-2);
angle2stress(1:n-2,3:n) = angle2stress(1:n-2,3:n) + 1/(2*l)*eye(n-2);

Coq = -diag(Diam(2:n-1))/2*angle2stress*K22^-1*K21;
Doq = zeros(n-2,n);

% Gyro measurement :
Cor = K22^(-1)*K21;
Dor = zeros(n,n);

% Accelerometer measurement
Coa = [inv(M)*K inv(M)*D];
Doa = inv(M)*Bo;
