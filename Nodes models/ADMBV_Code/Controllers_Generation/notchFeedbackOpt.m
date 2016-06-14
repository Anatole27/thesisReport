
%% Finds the optimal tuning for the notch architecture
%
% Author: Anatole VERHAEGEN
%
% After generating the notch filter, a tunable system will be generated and
% tuned with the Hinf structured technique. The model obtained is extracted
% from simulink.
% 
% INPUTS:
%
% * Win, Wout: I/O frequency matrix weights
% * inputs, outputs: I/O names
%
% OUTPUT:
%
% * CLnotch: closed-loop of the notch architecture

% Init gains
Kq = 0.0370;
Ki = 0.0034;
Kp = 3.9650e-05;

% Initiate Notch filter
w0 = 20*2*pi;
dw = 2*120/10;
s = zpk('s');
[lpZero, lpPole, lpK] = cheb2ap(2,40);
[An,Bn,Cn,Dn] = ssdata(zpk(lpZero, lpPole, lpK));
[An,Bn,Cn,Dn] = lp2bs(An,Bn,Cn,Dn,w0,dw);
notchFilter = ss(An,Bn,Cn,Dn);

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('notchFeedbackModel',{'PI','Kq'});

% Compute tunable model of closed-loop transfer 
tunable = getIOTransfer(ST0,inputs,outputs);

% Tune it
rng('shuffle')
opt = hinfstructOptions('Display','final','RandomStart',5);
weightedCL = hinfstruct(Wout*tunable*Win,opt);

% Get tunables
Kq = getBlockValue(weightedCL,'Kq');
Kq = Kq.d;
PI = getBlockValue(weightedCL,'PI');
Kp = PI.Kp;
Ki = PI.Ki;
tau = Kp/Ki;

% Get tuned system
load_system('notchFeedbackModel')
sys = ulinearize('notchFeedbackModel');
CLnotch = ss(sys.a,sys.b,sys.c,sys.d);
CLnotch.u = strrep(sys.InputName,'notchFeedbackModel/','');
CLnotch.y = strrep(sys.OutputName,'notchFeedbackModel/','');