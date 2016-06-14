%% Load model parameters
[EI,l,n,dm,Diam,S,rho,CL0,CLa,CLd,Cm0,Cma,Cmd,xAE,xCG,xFins,Jy,xIMU,CD0,kD,Dref,Vx,gamma,acc,X0,Z0] = loadModel_EB(100);
m = sum(dm);

%% Rigid missile flight dynamics
listAcc = [10 53 83 92];
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(S,rho,CL0,CLa,CLd,xFins,xAE,Cm0,Cma,Cmd,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,acc,l,dm,listAcc,Jy,xCG);

%% Init flexible body (no flight dynamics)
initFlexibleModelFullIO

%% Flexible missile
% Actuator dynamics
wT = 2*pi*25;
zT = 0.707;
wF = 2*pi*50;
zF = 0.707;

% Extract model from simulink
load_system('flexibleMissileFullIO');
load_system('missileSystem');
sys = ulinearize('flexibleMissileFullIO');
missile = ss(sys.a,sys.b,sys.c,sys.d);
missile.u = strrep(sys.InputName,'flexibleMissileFullIO/','');
missile.y = strrep(sys.OutputName,'flexibleMissileFullIO/','');