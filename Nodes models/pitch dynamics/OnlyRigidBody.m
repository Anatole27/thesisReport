[EI,l,n,dm,Diam,qS,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(5);
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(qS,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,l,dm);
return
%% Get system
A = Arb; B = Brb; C = Crb; D = Drb;
rbsys = ss(A,B,C,D);
rbsys.u = names_rb.u;
rbsys.y = names_rb.y;
rbsys.StateName = names_rb.x;

%% Adding lateral acceleration reference to system input and output

n_states = length(A);
B = [B zeros(n_states,1)];
C = [C; zeros(1,n_states)];
D = blkdiag(D,1);
names_rb.u = [names_rb.u; 'a_{zref}'];
names_rb.y = [names_rb.y; 'a_{zref}'];

%% Adding azref-az to output

r_az = find(strcmp(names_rb.y,'a_z'),1); % Lateral accel of rigid-body
r_azref = find(strcmp(names_rb.y,'a_{zref}'),1); % Lateral accel of rigid-body

Cdiff = C(r_azref,:)-C(r_az,:);
Ddiff = D(r_azref,:)-D(r_az,:);

C = [C;Cdiff];
D = [D;Ddiff];

names_rb.y = [names_rb.y; 'a_{zref}-a_z'];

%% Adding thetaT to output

p_thetaT = find(strcmp(names_rb.u,'\theta_T'),1); % thetaT
[~,n_inputs] = size(D);
C = [C;zeros(1,n_states)];
D = [D;zeros(1,n_inputs)];
D(end,p_thetaT) = 1;

names_rb.y = [names_rb.y; '\theta_T'];

rbsys = ss(A,B,C,D);
rbsys.u = names_rb.u;
rbsys.y = names_rb.y;
rbsys.StateName = names_rb.x;

%% Prepare for Hinf
z = {'a_{zref}-a_z';'\theta_T'};
y = {'a_{zref}';'w'}; % a_zIMU is not used (need 2 measures to know a_z). Too difficult. Plus, a_zIMU also measures g
w = {'a_{zref}'};
u = {'\theta_T'};
rbHinf = rbsys([z;y],[w;u]);
rbHinf = minreal(rbHinf);
[Ainf,Binf,Cinf,Dinf] = ssdata(rbHinf);
rbSim = rbsys([rbsys.y; y],[rbsys.u; u]);

% Weights
[r,p] = size(rbHinf);
s = zpk('s');
tauMax = 0.001;
addPole = 1/(1 + tauMax*s);

%in
Win = zpk(eye(p)); Win.u = rbHinf.u; Win.y = rbHinf.u;

%out
tau_az = 0.1;
%w_daz = 1/( 1/10*(1 + tau_az*s) ); % Out diff
w_daz = zpk(1/( 1 ));
tau_theta = 0.01;
w_thetaT = zpk(1/( 10 )); % Out control
%w_thetaT = 1/( 0.01/(1 + tau_az*s) )*addPole;
Wout = zpk(eye(r)); Wout.u = rbHinf.y; Wout.y = rbHinf.y;
Wout('a_{zref}-a_z','a_{zref}-a_z') = w_daz;
Wout('\theta_T','\theta_T') = w_thetaT;

rbHinfw = Wout*rbHinf*Win;

%% Hinf design
[K,CL,GAM,INFO] = hinfsyn(rbHinfw,length(y),length(u));
rbCL = lft(rbHinf,K);

% %% Sim
% dt = 0.01;
% time = (0:dt:10)';
% nstep = length(time);
% stepin = ones(nstep,1);
% impin = zeros(nstep,1); impin(1) = 0.1;
% sine = sin(time*10);
% input = sine;
% slowmo = 1;
% 
% [y,~] = lsim(rbCL(:,'a_{zref}'),input,time);
% thetaT = input;
% z = y(:,1); x = y(:,2);
% theta = y(:,3); w = y(:,4);
% u = y(:,5);
% w_nodes = zeros(nstep,10);
% 
% displaySim(0,0,0,alph0,Vx,xCG,thetaT0,thetaT,T0,z,x,theta,w,u,w_nodes,time,slowmo,10);