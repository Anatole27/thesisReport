function [Mm,Dm,Km,Bm,Cmq,Cmr,Cma,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coq,Cor,Coa,n_modes)

% Author : Anatole VERHAEGEN
% This function formulate a second-order nodal structural model into a 
% second-order MODAL structural model.
% Rigid-body modes are truncated and high fq modes
%
% Inputs :
% M,D,K : Mass, Damping and Stiffness matrices
% Bo : input matrix
% Coq,Cor,Coa : Output x-skin displacement,velocity and accel matrices
% n_modes : number of rigid-body modes kept
%
% Outputs :
% Mm, Dm, Km : Modal Mass, Damping and Stiffness  matrices
% om : vector of the natural frequencies (rad/s)
% Phi : Eigen vectors / Modes shape
% z : vector of Modal damping matrix
% Cmq, Cmr, Cma : Modal Output matrices

% Modes
[Phi,Om2] = eig(K,M);

% Remove rigid-body z-rotation and y-translation of the missile
Phi = Phi(:,3:n_modes+2);
Om2 = Om2(3:n_modes+2,3:n_modes+2);

% Modal realization
Mm = Phi'*M*Phi;
Km = Phi'*K*Phi;
Dm = Phi'*D*Phi;

% I/O matrices change
Bm = inv(Mm)*Phi'*Bo;
Cmq = Coq*Phi;
Cmr = Cor*Phi;
Cma = Coa*blkdiag(Phi,Phi);

% nat freq and damping
Omega = sqrt(Om2);
Z = 0.5*inv(Mm)*Dm*inv(Omega);
om = diag(Omega);
z = diag(Z);