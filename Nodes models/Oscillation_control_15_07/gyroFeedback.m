%% Control design
% 1st loop : 1st struct mode damping
%sisotool(flexiMissile('\Delta_q','\delta_{Fref}'))
Kdq = 0.24;
loopD = feedback(flexiMissile,Kdq,2,12);

% 2nd loop : SPPO damping
%sisotool(-loopD('q_{83}','\theta_{Tref}'))
Kq = -0.022;
loopDQ = feedback(loopD,Kq,1,7);

% 3d loop : PI on azref-az
%sisotool(loopDQ('a_{z83}','\theta_{Tref}'))
corrAz = 0.00023*tf([0.19 1],[1 0]);
loopDQA = feedback(loopDQ*corrAz,1,1,4);

%% Prepare for assessment
sys = linmod('vibeSupp');
CLVibeSupp = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeSupp.u = {'azref'};
CLVibeSupp.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};