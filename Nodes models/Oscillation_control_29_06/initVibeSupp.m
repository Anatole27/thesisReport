%% Load model parameters
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(100);

%% Rigid missile flight dynamics
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(S,rho,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,l,dm);

%% Init flexible body (no flight dynamics)
initFlexibleModel

%% Flexible missile
% Actuator dynamics
tau_act = 1/(2*pi*25);

% Extract model from simulink
sys = linmod('flexibleMissile');
flexiMissile = ss(sys.a,sys.b,sys.c,sys.d);
flexiMissile.u = {'thetaTref'};
flexiMissile.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon'};

%% Control design
% 1st loop : 1st struct mode damping
%sisotool(flexiMissile('epsilon','thetaTref'))
Keps = 2300;
loopEps = feedback(flexiMissile,Keps,1,9); 
% Weird that it damps 1st struct mode. Should increase fq if Keps > 0

% 2nd loop : SPPO damping
%rlocus(-loopEps('q','thetaTref'))
Kq = -0.31;
loopEpsQ = feedback(loopEps,Kq,1,1);

% 3d loop : PI on azref-az
%sisotool(loopEpsQ(2,1))
corrAz = 0.03*tf([0.44 1],[1 0]);
loopEpsQA = feedback(loopEpsQ*corrAz,1,1,2);
damp(loopEpsQA)

%% Prepare for assessment
sys = linmod('vibeSupp');
CLVibeSupp = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeSupp.u = {'azref'};
CLVibeSupp.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};
