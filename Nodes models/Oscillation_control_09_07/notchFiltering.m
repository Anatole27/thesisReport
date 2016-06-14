% Init
initVibeSupp
%% Control design
% If no low pass filter, unstable vibe mode due to q feedback
% Notch filter
w0 = 20*2*pi;
dw = 2*120/10;
s = zpk('s');
[lpZero, lpPole, lpK] = cheb2ap(2,20);
[A,B,C,D] = ssdata(zpk(lpZero, lpPole, lpK));
[A,B,C,D] = lp2bs(A,B,C,D,w0,dw);
notchFilter = ss(A,B,C,D);
%bode(notchFilter,logspace(1,3,1000));
notchFilter.u = '\theta_{Tref}';
filteredMissile = flexiMissile*notchFilter;
bode(flexiMissile('q_{83}',1),filteredMissile('q_{83}',1));
figure
bode(flexiMissile('a_{z83}',1),filteredMissile('a_{z83}',1));

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
