%% Load model parameters
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xAE,xCG,Jy,xIMU,CD0,kD,Dref,Vx,gamma,acc,X0,Z0] = loadModel_EB(100);
m = sum(dm);
%% Rigid missile flight dynamics
listAcc = [10 54 83 92];
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(S,rho,CL0,CLa,Cm0,Cma,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,acc,l,dm,listAcc);

%% Init flexible body (no flight dynamics)
initFlexibleModel

%% Flexible missile
% Actuator dynamics
wAct = 2*pi*25;
zAct = 0.707;

% Extract model from simulink
sys = linmod('flexibleMissile');
flexiMissile = ss(sys.a,sys.b,sys.c,sys.d);
flexiMissile.u = strrep(sys.InputName,'flexibleMissile/','');
flexiMissile.y = strrep(sys.OutputName,'flexibleMissile/','');