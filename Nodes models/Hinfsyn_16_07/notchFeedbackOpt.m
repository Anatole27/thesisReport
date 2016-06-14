%% Find the optimal tuning
% notch feedback

% Get weights
weightsHinfstruct;

% Init gains
Kq = 0.0370;
Ki = 0.0034;
Kp = 3.9650e-05;

% Initiate Notch filter
w0 = 20*2*pi;
dw = 2*120/10;
s = zpk('s');
[lpZero, lpPole, lpK] = cheb2ap(2,20);
[An,Bn,Cn,Dn] = ssdata(zpk(lpZero, lpPole, lpK));
[An,Bn,Cn,Dn] = lp2bs(An,Bn,Cn,Dn,w0,dw);
notchFilter = ss(An,Bn,Cn,Dn);

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('notchFeedbackModel',{'PI','Kq'});

% Compute tunable model of closed-loop transfer 
tunable = getIOTransfer(ST0,{'a_{zref}'},{'a_{zerr}','\theta_T'});

% Tune it
rng(0)
opt = hinfstructOptions('Display','final','RandomStart',5);
weightedCL = hinfstruct(blkdiag(WErr,WTheta)*tunable,opt);

% Get tunables
Kq = getBlockValue(weightedCL,'Kq');
Kq = Kq.d;
PI = getBlockValue(weightedCL,'PI');
Kp = PI.Kp;
Ki = PI.Ki;

% Get tuned system
load_system('notchFeedbackModel')
sys = ulinearize('notchFeedbackModel');
CLnotch = ss(sys.a,sys.b,sys.c,sys.d);
CLnotch.u = strrep(sys.InputName,'notchFeedbackModel/','');
CLnotch.y = strrep(sys.OutputName,'notchFeedbackModel/','');

% Assessement
% close all
% bode(CLnotch('a_{zerr}','a_{zref}'),1/WErr);
% figure
% bode(CLnotch('\theta_T','a_{zref}'),1/WTheta);
% figure
% step(CLnotch('a_{zerr}','a_{zref}'));