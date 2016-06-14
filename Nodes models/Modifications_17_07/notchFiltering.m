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
notchFilter.u = '\theta_{Tref}';
filteredMissile = flexiMissile*notchFilter;

% 2nd loop : SPPO damping
%sisotool(-filteredMissile('q_{83}','\theta_{Tref}'))
Kq = -0.022;
loopNotchQ = feedback(filteredMissile,Kq,1,7);

% 3d loop : PI on azref-az
%sisotool(loopDQ(4,1))
corrAz = 0.00019*tf([0.1 1],[1 0]);
loopNotchQA = feedback(loopNotchQ*corrAz,1,1,4);

%% Assessment
sys = linmod('vibeNotch');
CLVibeNotch = ss(sys.a,sys.b,sys.c,sys.d);
CLVibeNotch.u = {'azref'};
CLVibeNotch.y = {'q_meas';'az_meas';'alpha';'thetaT';'q_noise';'az_noise';
    'q';'az';'epsilon';'error'};
