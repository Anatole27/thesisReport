function [Mm,Dm,Km,Bm,Cmz,Cmv,Do,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coz,Cov,Do,n_modes,m,xCG,Jy,l,n)

% Author : Anatole VERHAEGEN
% This function formulate a second-order nodal structural model into a 
% second-order MODAL structural model.
% Rigid-body modes are truncated and high fq modes
%
% Inputs :
% M,D,K : Mass, Damping and Stiffness matrices
% Bo : input matrix
% Coz,Cov : Output matrix [strain, gyro , acc]'
% Do : Feedforward matrix [strain, gyro , acc]'
% n_modes : number of rigid-body modes kept
% m : mass of missile
% xCG : centre of gravity position
% Jy : rotational inertia of missile about CG
% l : beam element length

%
% Outputs :
% Mm, Dm, Km : Modal Mass, Damping and Stiffness  matrices
% om : vector of the natural frequencies (rad/s)
% Phi : Eigen vectors / Modes shape
% z : vector of Modal damping matrix
% Cmz : Modal Output displacement  matrix [strain, gyro, acc]
% Cmv : Modal Output displacement  matrix [strain, gyro, acc]
% Do : Modal feedforward matrix [strain gyro acc]'

% Modes
[Phi,Om2] = eig(K,M);

% Remove rigid-body z-rotation and y-translation of the missile
Phi = Phi(:,3:n_modes+2);
Om2 = Om2(3:n_modes+2,3:n_modes+2);

% Modal realization
Mm = Phi'*M*Phi;
Km = Phi'*K*Phi;
Dm = Phi'*D*Phi;

%% I/O matrices change
Bm = inv(Mm)*Phi'*Bo;
Cmz = Coz*Phi;
Cmv = Cov*Phi;

% Solid feedforward matrices at nodes
distCG = xCG - linspace(0,(n-1)*l,n)';
Doa_rb = 1/m*eye(n) + 1/Jy*distCG*distCG';

% Feedforward matrix
Do = Do-[zeros(n-2,n); zeros(n,n); Doa_rb]; % Suppress rb dynamics

%% nat freq and damping
Omega = sqrt(Om2);
Z = 0.5*inv(Mm)*Dm*inv(Omega);
om = diag(Omega);
z = diag(Z);