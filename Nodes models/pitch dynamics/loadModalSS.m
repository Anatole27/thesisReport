clear all
% Load missile parameters
[EI,l,n,dm,Diam,qS,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(5);

%% Flexible body model
% Get 2nd order structural model
[M,D,K,Bo,Coq,Cor,Coa,Dq,Dr,Da] = getMDK_Leckie(EI,l,n,dm,Diam);

% Put into modal rep without rigid body modes
n_modes = 1;
[Mm,Dm,Km,Bm,Cmq,Cmr,Cmacc,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coq,Cor,Coa,n_modes);

% Put into modal SS form 2 (Xi = [qmi, qmoi], qmoi = zetai*qmi +
% qmi_dot/omegai) (Flexible Body)
[Afb,Bfb,Cfb,Dfb,Vm,names_fb] = getSSForm2(om,z,Bm,Cmq,Cmr,Cmacc,Dq,Dr,Da,n,Phi);

%% Rigid-body model and trim
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(qS,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,l,dm);

%% Actuator placement on flexible body
% Connect lateral vectoring to tail
Bfb = [Bfb(:,1)*T0*cos(thetaT0),zeros(2*n_modes,1)];
[r,~] = size(Dfb);
Dfb = [Dfb(:,1)*T0*cos(thetaT0),zeros(r,1)];
names_fb.u = {'\theta_T','\delta\alpha_{gust}'}';

%% Merging flexible + rigid-body models
% Inputs
A = blkdiag(Arb,Afb);
B = [Brb; Bfb];
C = blkdiag(Crb,Cfb);
D = [Drb; Dfb];
names.u = names_rb.u;
names.y = [names_rb.y; names_fb.y];
names.x = [names_rb.x; names_fb.x];

%% Creating IMU measurements

iIMU = floor(xIMU/l)+1; % Get index of sensor package
r_az = find(strcmp(names.y,'a_zIMU'),1); % Lateral accel of rigid-body at IMU
r_q = find(strcmp(names.y,'q'),1); % Pitch rate of rigid-body
r_azNoise = find(strcmp(names.y,strcat('a_{z',num2str(iIMU),'}')),1); % structural lateral accel
r_qNoise = find(strcmp(names.y,strcat('q_{',num2str(iIMU),'}')),1); % structural pitch rate

Cy_IMU = C([r_az r_q],:) + C([r_azNoise r_qNoise],:);
Dy_IMU = D([r_az r_q],:) + D([r_azNoise r_qNoise],:);

% Add IMU measurement to model outputs
C = [C; Cy_IMU];
D = [D; Dy_IMU];
names.y = [names.y; 'a_{zmeas}';'q_{meas}'];

%% Adding lateral acceleration reference to system input and output

n_states = length(A);
B = [B zeros(n_states,1)];
C = [C; zeros(1,n_states)];
D = blkdiag(D,1);
names.u = [names.u; 'a_{zref}'];
names.y = [names.y; 'a_{zref}'];

%% Adding azref-az to output

r_az = find(strcmp(names.y,'a_z'),1); % Lateral accel of rigid-body
r_azref = find(strcmp(names.y,'a_{zref}'),1); % Lateral accel of rigid-body

Cdiff = C(r_azref,:)-C(r_az,:);
Ddiff = D(r_azref,:)-D(r_az,:);

C = [C;Cdiff];
D = [D;Ddiff];

names.y = [names.y; 'a_{zref}-a_z'];

%% Adding thetaT to output

p_thetaT = find(strcmp(names.u,'\theta_T'),1); % thetaT
[~,n_inputs] = size(D);
C = [C;zeros(1,n_states)];
D = [D;zeros(1,n_inputs)];
D(end,p_thetaT) = 1;

names.y = [names.y; '\theta_T'];

%% Compute system
missile = ss(A,B,C,D);
missile.u = names.u;
missile.y = names.y;
missile.StateName = names.x;