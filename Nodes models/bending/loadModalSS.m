% Load missile parameters
[EI,qS,Vx,CL,CN,l,n,dm,Diam] = loadModel_EB(n)

% Get 2nd order structural model
[M,D,K,Bo,Coq,Cor] = getMDK_Leckie(EI,qSCLa,l,n,dm,Diam);

% Put into modal rep without rigid body modes
[Mm,Dm,Km,Bm,Cmq,Cmr,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coq,Cor);

% Put into modal SS form 2 (Xi = [qmi, qmoi], qmoi = zetai*qmi +
% qmi_dot/omegai) (Flexible Body)
[Afb,Bfb,Cfb,Dfb,Vm] = getSSForm2(om,z,Bm,Cmq,Cmr);

% Find trim state of missile
%[

% Add rigid-body states (y,y_dot and psi, r)
%[Arb,Brb,Crb,Drb] = getRigidBodySS(EI,qSCLa,l,n,dm,x_CG)

%% Actuator placement

% Get actuator and mode weights
w_act = getActuatorWeights(n);
w_mode = getModeWeights(n-2);

% Get Hinf actuator placement index
[Sigma_act,corr_act] = getHinfActPlacementIndex(w_act,w_mode,B,C,om,z);

% Select s2 1st actuators
s1 = floor(n/3);
n_modes = 5;
actuatorList = placementSelection(s1,Sigma_act(1:n_modes,:));

% Select s3 2nd actuators
eps = 0.06;
actuatorList = correlationSelection(eps,actuatorList,corr_act,Sigma_act);

% Applying only selected actuators
% I don't do this now, because input forces are not only control forces. It
% can also be disturbance forces. Therefore all input force at each node
% are taken into account for the sensor placement

%% Sensor placement

% Get sensors
w_sens_strain = getSensorWeightsStrain(n-2);
w_sens_gyr = getSensorWeightsGyr(n);

p_strain = length(w_sens_strain);
p_gyr = length(w_sens_gyr);

% Get Hinf actuator placement index
[Sigma_sens_strain,corr_sens_strain] = getHinfSensPlacementIndex(w_sens_strain,w_mode,B,C(1:p_strain,:),om,z);
[Sigma_sens_gyr,corr_sens_gyr] = getHinfSensPlacementIndex(w_sens_gyr,w_mode,B,C(p_strain+1:p_strain+p_gyr,:),om,z);

% Select r2 1st sensors
r1 = floor(n/3);
sensorList_strain = placementSelection(r1,Sigma_sens_strain(1:n_modes,:));
sensorList_gyr = placementSelection(r1,Sigma_sens_gyr(1:n_modes,:));

% Select s3 2nd actuators
eps = 0.06;
sensorList_strain = correlationSelection(eps,sensorList_strain,corr_sens_strain,Sigma_sens_strain);
sensorList_gyr = correlationSelection(eps,sensorList_gyr,corr_sens_gyr,Sigma_sens_gyr);
sensorList = sensorList_strain;

%B_w = B(:,[i_thruster,i_PIF]); % Lateral thrust + pif paf


%C_z = C(sensorList,:);
Cy = C(sensorList,:);
%B = [B_w, B_u];
%C = [C_z;C_y];
[~,p] = size(B);
[r,~] = size(C);

%% Order reduction
G_full = ss(A,B,C,zeros(r,p));
state_kept = 1:2*n_modes;
A = A(state_kept,state_kept);
B = B(state_kept,:);
C = C(:,state_kept);
D = zeros(r,p);
G_red = ss(A,B,C,D);
Vm = Vm([1:n_modes (n-2)+1:(n-2)+n_modes],state_kept);
Phi = Phi(:,1:n_modes);