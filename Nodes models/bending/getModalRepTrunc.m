function [Mm,Dm,Km,Bm,Cmq,Cmr,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coq,Cor)

% Author : Anatole VERHAEGEN
% This function formulate a second-order nodal structural model into a 
% second-order MODAL structural model.
% Rigid-body modes are truncated !
%
% Inputs :
% M,D,K : Mass, Damping and Stiffness matrices
% Bo : input matrix
% Coq,Cor : Output displacement and velocity matrices
%
% Outputs 
% Mm, Dm, Km : Modal Mass, Damping and Stiffness  matrices
% om : vector of the natural frequencies (rad/s)
% Phi : Eigen vectors / Modes shape
% z : vector of Modal damping matrix
% Cmq, Cmq : Modal Output matrices

% Modes
[Phi,Om2] = eig(K,M);

% Remove integrators/z-rotation and y-translation of the missile
Phi = Phi(:,3:end);
Om2 = Om2(3:end,3:end);

% Modal rep
Mm = Phi'*M*Phi;
Km = Phi'*K*Phi;
Dm = Phi'*D*Phi;

% 
Bm = inv(Mm)*Phi';
Cmq = Coq*Phi;
Cmr = Cor*Phi;
Omega = sqrt(Om2);
Z = 0.5*inv(Mm)*Dm*inv(Omega);
om = diag(Omega);
z = diag(Z);