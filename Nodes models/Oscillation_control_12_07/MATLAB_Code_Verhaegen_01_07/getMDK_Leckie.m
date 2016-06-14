function [M,D,K,Bo,Coq,Cor,Coa,Doq,Dor,Doa] = getMDK_Leckie(EI,l,n,dm,Diam)

% Author : Anatole VERHAEGEN
% This function computes M,D,K matrices of the second-order structural
% model plus input and output matrices
% M uddot + D udot + K u = [Forces; Moments]
% u = [displacement; local pitch angle]
%
% Inputs :
% EI : stiffness along the beam, (n-1 x 1)
% l : length of an element of beam
% n : number of nodes
% dm : mass vector (n x 1)
% Diam : diameter along the structure (n x 1)

% Outputs
% M : mass matrix (n x n)
% D : damping matrix (n x n)
% K : stiffness matrix (n x n)
% Bo : input matrix (n x p)
% Coq : output displacement matrix (r_q x n)
% Cor : output rate matrix (r_v x n)
% Coa : output acceleration matrix (r_a x n)
% Doq : feedforward displacement
% Dor : feedforward rate
% Doa : feedforward acceleration

% Stiffness matrix
% K = [K11 K12; K21 K22]

K11 = zeros(n); K22 = K11; K12 = K11;

for i = 1:n-1
    % For one element beam
    k11 = 12*EI(i)/l^3*[1 -1; -1 1];
    k12 = 6*EI(i)/l^2*[-1 -1; 1 1];
    k22 = EI(i)/l*[4 2; 2 4];
    
    % For the complete missile
    K11(i:i+1,i:i+1) = K11(i:i+1,i:i+1) + k11;
    K12(i:i+1,i:i+1) = K12(i:i+1,i:i+1) + k12;
    K22(i:i+1,i:i+1) = K22(i:i+1,i:i+1) + k22;
end

K21 = K12'; % Because K is symmetric

% New K : u = [displacement] and [local pitch angle] = K22^(-1)*K21*[displacement].
% (Hyp : no inertia, no local torque)
K = K11-K12*inv(K22)*K21;
K = (K+K')/2; % Make K symmetric to correct computation approximations

% Damping matrix
D = 1/6000*K; % Makes 1st structural mode damping = 1%

% Mass matrix
M = diag(dm);

% Input
Bo = eye(n);

% Stress measurement :
% I use the term stress, but actually, this is the local elongation on the
% upper skin epsilon_x which is calculated, not a stress.
% epsilon_x = "stress" = -dlocalPitchAngle/dx*Diam/2
% dlocalPitchAngle/dx ~ (localPitchAngle(i+1) - localPitchAngle(i-1))/(2*l)
% angle2stress is created to have epsilon_x = angle2stress*localPitchAngle.
% epsilon_x are available at nodes 2 to n-1. Not at extremities.
angle2stress = zeros(n-2,n);
angle2stress(1:n-2,1:n-2) = 1/(2*l)*diag(Diam(2:n-1)/2);
angle2stress(1:n-2,3:n) = angle2stress(1:n-2,3:n) - 1/(2*l)*diag(Diam(2:n-1)/2);

Coq = angle2stress*K22^-1*K21; % Because localPitchAngle =  K22^(-1)*K21*[displacements]
Doq = zeros(n-2,n);

% Gyro measurement :
Cor = K22^(-1)*K21; % Because localPitchRate = K22^(-1)*K21*[lateralSpeeds]
Dor = zeros(n,n);

% Accelerometer measurement
Coa = [-inv(M)*K -inv(M)*D]; % True with rigid-body modes. False when truncated
Doa = inv(M)*Bo;