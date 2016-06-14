function [flexibleBody,Vm] = getFlexibleBodySS(EI,l,n,dm,Diam,m,xCG,Jy,n_modes)
%% Generates the flexible-body state-space system of a missile.
%
% Author : Anatole VERHAEGEN
%
% The first step is to find the second order structural matrices M, D and K
% using getMDKLeckie. Then the structural system is formulated under its 
% modal form keeping only a few flexible-body modes with getModalRepTrunc.
% Finally after adding uncertainty on the natural frequencies and the 
% damping ratios, the system is formulated under its state-space form 2 
% with getSSForm2.
%
% INPUTS:
% 
% * EI: bending stiffness vector of beams [n-1] (N.m2)
% * l: element beam length (m)
% * n: number of nodes 
% * dm: nodes masses vector [n] (kg)
% * Diam: diameters vector [n] (m)
% * m: total mass (kg)
% * xCG: CG position (m)
% * Jy: rotational inertia at CG about y-axis (kg.m2)
% * n_modes: number of bending modes considered
%
% OUTPUTS:
%
% * flexibleBody: uncertain SS system of flexible body
% * Vm: transformation matrix of modal form 2 to original SS

% Get 2nd order structural model
[M,D,K,Bo,Coz,Cov,Do] = getMDKLeckie(EI,l,n,dm,Diam);

% Put into modal representation without rigid-body modes (X = [qm,qm_dot])
% High order modes are truncated
[Mm,Dm,Km,Bm,Cmz,Cmv,Do,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coz,Cov,Do,...
    n_modes,m,xCG,Jy,l,n);

%% Add uncertainty on frequency
[om,z] = addUncertaintyFB(om,z);

% Put into modal SS form 2 (Gawronski 2004) 
% Xi = [qmi, qmoi] with qmoi = zetai*qmi + qmi_dot/omegai. 
% zetai is the damping of mode i, omegai its natural fq
[flexibleBody,Vm] = getSSForm2(om,z,Bm,Cmz,Cmv,Do,n,Phi);

%% Actuator placement and sensor placement
% Only keep useful I/O (Force at node 1, pitch rate and latax at IMU,
% strain gage measurement at node 50)
flexibleBody = flexibleBody({'\varepsilon_{46}';'q_{10}';'q_{83}';...
    'a_{z10}';'a_{z53}';'a_{z83}';'a_{z92}';'z_{m1}'},...
    {'F_{1}','F_{50}'});
