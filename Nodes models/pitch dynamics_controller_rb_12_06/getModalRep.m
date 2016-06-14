function [Mm,Dm,Km,Bm,Cmq,Cmr,om,Phi,z] = getModalRep(M,D,K,Bo,Coq,Cor)

% Author : Anatole VERHAEGEN
% This function formulate a second-order nodal structural model into a 
% second-order MODAL structural model
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

% Nodal SS rep
% A = [zeros(n), eye(n); -inv(M)*K, -inv(M)*D];
% B = [zeros(n); inv(M)*Bo];
% C = [Coq Cor];

% % Modal rep
 %[Am,Bm,Cm,Vm] = realdiag(A,B,C);


% Modes
[Phi,Om2] = eig(K,M);

% Remove integrators/z-rotation and y-translation of the missile
% Phi = Phi(:,3:n);
% Om2 = Om2(3:n,3:n);

% 
% % figure
% % hold on
% % for i = 1:4
% %     plot(modes(:,i))
% %     eigen(i,i)
% % end
% 

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
if(Om2(3,3)/Om2(2,2) < 10^6)
    warning('Natural frequencies highly different. Try getModalRepTrunc instead to get rid of rigid-body modes.');
end

om = diag(Omega);
z = diag(Z);