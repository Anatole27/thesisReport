%% Load model parameters
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(100);
[Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names_rb] = getRigidBodySS(S,rho,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,Vx,X0,Z0,gamma,l,dm);

% Actuator dynamics
tau_act = 1/(2*pi*25);

%% Init flexible body
initFlexibleModel

%% Flexible missile
sys = linmod('flexibleMissile');
flexiMissile = ss(sys.a,sys.b,sys.c,sys.d);
flexiMissile.u = {'thetaTref'};
flexiMissile.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon'};

%% Control design
% If no low pass filter, unstable vibe mode due to q feedback
% Notch filter
w0 = 120;
zeta0 = 0.1;
zeta1 = 1;
s = zpk('s');
notchFilter = (1 + 2*zeta0/w0*s + s^2/w0^2)^2 / (1 + 2*zeta1/w0*s + s^2/w0^2)^2;
notchFilter.u = 'thetaTref';
filteredMissile = flexiMissile*notchFilter;
%bode(flexiMissile(1,1),filteredMissile(1,1));

%sisotool(-filteredMissile('q','thetaTref'))
Kq2 = -0.633;
loopNotchQ = feedback(filteredMissile,Kq2,1,1);

%sisotool(loopNotchQ(2,1))
corrAz2 = 0.15*tf([0.1 1],[1 0]);
loopNotchQA = feedback(loopNotchQ*corrAz,1,1,2);

%% Assessment
sys = linmod('vibeNotch');
CLVibeNotch = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeNotch.u = {'azref'};
CLVibeNotch.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};
