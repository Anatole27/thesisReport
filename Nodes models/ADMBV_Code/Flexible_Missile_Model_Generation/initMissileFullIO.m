%% Load model parameters
display('Loading parameters')
[EI,l,n,dm,Diam,S,rho,CL0,CLa,CLd,Cm0,Cma,Cmd,xAE,xCG,xFins,Jy,CD0,kD,Dref,V0,gamma,acc] = loadModelEB(100);
m = sum(dm);

%% Rigid missile flight dynamics
display('Rigid-body SS generation')
listAcc = [10 53 83 92]; % list of nodes equipped with accelerometers
[rigidBodySS,alph0,T0,thetaT0] = getRigidBodySS(S,rho,CL0,CLa,CLd,xFins,xAE,Cm0,Cma,Cmd,CD0,kD,Dref,V0,gamma,acc,l,dm,listAcc,Jy,xCG);

%% Flexible body
display('Flexible-body SS generation')
n_modes = 5; % Number of bending modes kept
[flexibleBodySS,Vm] = getFlexibleBodySS(EI,l,n,dm,Diam,m,xCG,Jy,n_modes);

%% Actuators dynamics 
% Thrust vectoring
wT = 2*pi*25;
zT = 0.707;

% Fins
wF = 2*pi*50;
zF = 0.707;

%% Flexible missile

display('Extracting flexible missile model from simulink')
% Extract model from simulink
load_system('flexibleMissileFullIO');
load_system('missileSystem');
sys = ulinearize('flexibleMissileFullIO');
missile = ss(sys.a,sys.b,sys.c,sys.d);
missile.u = strrep(sys.InputName,'flexibleMissileFullIO/','');
missile.y = strrep(sys.OutputName,'flexibleMissileFullIO/','');
