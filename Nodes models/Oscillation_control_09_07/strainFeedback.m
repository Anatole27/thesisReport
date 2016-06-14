% Init
initVibeSupp
%% Control design
% 1st loop : 1st struct mode damping
%sisotool(-flexiMissile('\epsilon_{46}','\theta_{Tref}'))
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