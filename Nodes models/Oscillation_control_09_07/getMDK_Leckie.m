function [M,D,K,Bo,Coz,Cov,Do] = getMDK_Leckie(EI,l,n,dm,Diam)

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
% Coz : output displacement matrix [strain, gyro, accelerometer]'
% Cov : output speed matrix [strain, gyro, accelerometer]'
% Doq : feedforward matrix [strain, gyro, accelerometer]'

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

% New K : u = [displacement] and [local pitch angle] = -K22^(-1)*K21*[displacement].
% (Hyp : no inertia, no local torque)
K = K11-K12*inv(K22)*K21;
K = (K+K')/2; % Make K symmetric to correct computation approximations

% Damping matrix
D = 1/6000*K; % Makes 1st structural mode damping = 1%

% Mass matrix
M = diag(dm);

% Input
Bo = eye(n);

% Strain measurement :
% Local elongation on the upper skin epsilon_x is calculated.
% epsilon (strain) = -dlocalPitchAngle/dx*Diam/2
% dlocalPitchAngle/dx ~ (localPitchAngle(i+1) - localPitchAngle(i-1))/(2*l)
% angle2eps is created to have epsilon = angle2eps*localPitchAngle.
% epsilon are available at nodes 2 to n-1. Not at extremities.
angle2eps = zeros(n-2,n);
angle2eps(1:n-2,1:n-2) = 1/(2*l)*diag(Diam(2:n-1)/2);
angle2eps(1:n-2,3:n) = angle2eps(1:n-2,3:n) - 1/(2*l)*diag(Diam(2:n-1)/2);

Cozeps = -angle2eps*K22^-1*K21; % Because localPitchAngle =  -K22^(-1)*K21*[displacements]
Coveps = zeros(n-2,n);
Doeps = zeros(n-2,n);

% Gyro measurement :
Cozq = zeros(n,n); % Because localPitchRate = -K22^(-1)*K21*[lateralSpeeds]
Covq = -K22^(-1)*K21;
Doq = zeros(n,n);

% Accelerometer measurement
Coza = -inv(M)*K;
Cova = -inv(M)*D;
Doa = inv(M)*Bo;

% Concatenation
Coz = [Cozeps; Cozq; Coza];
Cov = [Coveps; Covq; Cova];
Do = [Doeps; Doq; Doa];