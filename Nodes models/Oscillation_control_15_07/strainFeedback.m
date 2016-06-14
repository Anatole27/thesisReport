%% Control design
% 1st loop : 1st struct mode damping
%sisotool(-flexiMissile('\epsilon_{46}','\delta_{Fref}'))
Keps = -600 ;
loopEps = feedback(flexiMissile,Keps,2,9);

% 2nd loop : SPPO damping
%sisotool(-loopEps('q_{83}','\theta_{Tref}'))
Kq = -0.011;
loopEpsQ = feedback(loopEps,Kq,1,7);

% 3d loop : PI on azref-az
%sisotool(loopEpsQ('a_{z83}','\theta_{Tref}'))
corrAz = 0.00005*tf([0.1 1],[1 0]);
loopEpsQA = feedback(loopEpsQ*corrAz,1,1,4);

%% Prepare for assessment
sys = linmod('vibeSupp');
CLVibeSupp = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeSupp.u = {'azref';'\delta_Fref'};
CLVibeSupp.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error';'';'';'';'';''};