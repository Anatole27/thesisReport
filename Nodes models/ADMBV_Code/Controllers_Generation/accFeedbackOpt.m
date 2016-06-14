%% Finds the optimal tuning for the acc architecture
%
% Author: Anatole VERHAEGEN
%
% A tunable system will be generated and
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
% * CLacc: closed-loop of the acc architecture

% Init gains
KSa = -0.16;
Kq = 0.0375;
Ki = 0.0034;
Kp = 3.9733e-05;

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('accFeedbackModel',{'PI','Kq'});

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
load_system('accFeedbackModel')
sys = ulinearize('accFeedbackModel');
CLacc = ss(sys.a,sys.b,sys.c,sys.d);
CLacc.u = strrep(sys.InputName,'accFeedbackModel/','');
CLacc.y = strrep(sys.OutputName,'accFeedbackModel/','');
CLacc = minreal(CLacc);