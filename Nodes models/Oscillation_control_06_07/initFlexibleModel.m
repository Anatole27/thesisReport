%% This script will use the missile characteristics to compute a state space realization of flexible body
% Author : Anatole VERHAEGEN

% Get 2nd order structural model
[M,D,K,Bo,Coz,Cov,Do] = getMDK_Leckie(EI,l,n,dm,Diam);

% Put into modal representation without rigid-body modes (X = [qm,qm_dot])
% High order modes are truncated
n_modes = 1; % number of structural modes kept
[Mm,Dm,Km,Bm,Cmz,Cmv,Do,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coz,Cov,Do,n_modes,m,xCG,Jy,l,n);

% Put into modal SS form 2 (Gawronski 2004) (Xi = [qmi, qmoi] with qmoi = zetai*qmi +
% qmi_dot/omegai. zetai is the damping of mode i, omegai its natural fq)
[Afb,Bfb,Cfb,Dfb,Vm,names_fb] = getSSForm2(om,z,Bm,Cmz,Cmv,Do,n,Phi);
flexi = ss(Afb,Bfb,Cfb,Dfb);
flexi.u = names_fb.u;
flexi.y = names_fb.y; 
om/(2*pi)
%% Actuator placement and sensor placement
% Only keep useful I/O (Force at node 1, pitch rate and latax at IMU,
% strain gage measurement at node 50)
flexi = flexi({'q_{75}';'a_{z75}';'\varepsilon_{50}'},'F_{1}');

% % Issue with low frequency latax. Should be 0. Static gain due to
% % rigid-body modes not perfectly erased.
% s = zpk('s'); 
% filter = s/(s+100); Wout = zpk(eye(3)); Wout(2,2) = filter;
% flexi = Wout*flexi;
