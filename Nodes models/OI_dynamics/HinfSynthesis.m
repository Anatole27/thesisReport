% Load Model
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(5);
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(S,rho,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,l,dm);

% Actuator dynamics
tau_act = 1/(2*pi*25);

% Weights
s = zpk('s');
Wz = zpk(10);
Wz = ss(Wz);

% Load augmented & weighted model
sysw = linmod('weightedSystem');
sysw = ss(sysw.a,sysw.b,sysw.c,sysw.d);
sysw.u = {'a_{zref}';'\theta_{Tref}'};
sysw.y = {'a_{zref}-a_z';'a_zIMU';'q';'a_{zref}'};
N_MEAS = 3;
N_CON = 1;

% Hinf synthesis
[K,CL,GAM,INFO] = hinfsyn(sysw,N_MEAS,N_CON);

% Load augmented model
sys = linmod('augSystem');
sys = ss(sys.a,sys.b,sys.c,sys.d);
rbCL = lft(sys,K);

step(sysw);