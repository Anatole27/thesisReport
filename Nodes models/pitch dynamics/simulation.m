% Closed Loop system
G_CL = lft(G_OL,K);

%% Time
slowmo = 80;
dt = 1/(24*slowmo);
tend = 0.4;
time = 0:dt:tend;

% Input
Fin = zeros(length(time),p);
Fin(:,1) = wgn(length(time),1,80);
%Fin(:,1) = 26000;

% Sim
y_CL = lsim(G_CL,Fin(:,1:pw),time);
y_OL = lsim(G_OL,Fin,time);

% Get nodes coordinates
q_CL = y_CL(:,1:n_modes)*Phi';
q_OL = y_OL(:,1:n_modes)*Phi';

% Get control forces
F_CL = y_CL(:,rz-pu:rz-1);
i_F = [i_thruster,actuatorList];

%% Display(
dualSim(q_OL,q_CL,Fin(:,1),F_CL,i_F,time,slowmo,n);
%displaySim(q_OL(:,1:n),time,slowmo,n);