function [generalizedSys,N_MEAS,N_CON,N_Z,N_W] = generalizeSystem(sys,epsList)
% Modifies the system "missile" to put it into generalized architecture :
%       ____________
% w1 ->|            |-> z1
% w2 ->|     G      |-> z2
% u  ->|____________|-> y

% w1 and z1 are the extra input and output that the controller design will
% not use. These are use for simulation
% w2 is the perturbations : azref in this case
% z2 is the performance vector : [az-azref] at least
% y is the control output : thetaT
% u is the control input : [az_meas,q_meas,eps]

% Inputs :
% sys is the missile system with all inputs and outputs available
% epsList is the list of nodes ID where the stress gages are placed

% Outputs :
% generalizedSystem is the generalized system (...)
% N_MEAS,N_CON,N_Z,N_W are the size of vectors (y,u,z2,w2)

z1name = sys.y;
w1name = {'\delta\alpha_{gust}'};
w2name = {'a_{zref}'};
z2name = {'a_{zref}-a_z';'\theta_T'};
uname = {'\theta_T'};
%y_struct = [epsilon_i]
epsName = {};
for iEps = epsList
    epsName = [epsName; strcat('\epsilon_{',num2str(iEps),'}')];
end

yname = [{'a_{zref}';'a_{zmeas}';'q_{meas}'}; epsName];

generalizedSys = sys([z1name;z2name;yname],[w1name;w2name;uname]);
N_MEAS = length(yname);
N_CON = length(uname);
N_Z = length(z2name);
N_W = length(w2name);