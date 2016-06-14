%% Control design
% 1st loop : 1st struct mode damping
%sisotool(flexiMissile('\Sum a_z',2))
Kaz = tf(0.16,[1 0]) ;
loopA = feedback(flexiMissile,Kaz,2,13);
loopA = minreal(loopA); % Get rid of numerical errors

% 2nd loop : SPPO damping
%sisotool(-loopA('q_{83}','\theta_{Tref}'))
Kq = -0.021;
loopAQ = feedback(loopA,Kq,1,7);

% 3d loop : PI on azref-az
%sisotool(loopAQ('a_{z83}','\theta_{Tref}'))
corrAz = 0.00026*tf([0.1 1],[1 0]);
loopAQA = feedback(loopAQ*corrAz,1,1,4);

%% Prepare for assessment
sys = linmod('vibeSupp');
CLVibeSupp = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeSupp.u = {'azref'};
CLVibeSupp.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};