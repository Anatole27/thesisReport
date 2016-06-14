%% Control design
% 1st loop : 1st struct mode damping
%sisotool(-flexiMissile('\epsilon_{46}',2))
Keps = -600 ;
loopEps = feedback(flexiMissile,Keps,2,9);
% Weird that it damps 1st struct mode. Should increase fq if Keps > 0

% 2nd loop : SPPO damping
%sisotool(-loopEps('q_{83}','\theta_{Tref}'))
Kq = -0.018;
loopEpsQ = feedback(loopEps,Kq,1,7);

% 3d loop : PI on azref-az
%sisotool(loopEpsQ(4,1))
corrAz = 0.00012*tf([0.1 1],[1 0]);
loopEpsQA = feedback(loopEpsQ*corrAz,1,1,4);

%% Prepare for assessment
sys = linmod('vibeSupp');
CLVibeSupp = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeSupp.u = {'azref'};
CLVibeSupp.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};