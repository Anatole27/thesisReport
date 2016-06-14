sys = flexiNodes;

%% Time
slowmo = 1000;
dt = 1/(24*slowmo);
tend = 0.2;
time = 0:dt:tend;

% Input
Fin = zeros(length(time),2);
%Fin(:,2) = wgn(length(time),1,80);
Fin(1,1) = 70000;

% Sim
y = lsim(sys,Fin,time);

% Get nodes coordinates
w = y(:,1:n_modes)*Phi';

%% Display(
%dualSim(q_OL,q_CL,Fin(:,1),F_CL,i_F,time,slowmo,n);
displaySim(w(:,1:n),time,slowmo,n,1,'simFastLeft');