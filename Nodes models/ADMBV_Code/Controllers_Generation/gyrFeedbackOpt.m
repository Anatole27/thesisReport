%% Finds the optimal tuning for the gyro architecture
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
% * CLgyro: closed-loop of the gyro architecture

% Init gains
Kdq = -0.24;
Kq = 0.0378;
Ki = 0.0034;
Kp = 3.6076e-05;

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('gyrFeedbackModel',{'PI','Kq'});

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
load_system('gyrFeedbackModel')
sys = ulinearize('gyrFeedbackModel');
CLgyr = ss(sys.a,sys.b,sys.c,sys.d);
CLgyr.u = strrep(sys.InputName,'gyrFeedbackModel/','');
CLgyr.y = strrep(sys.OutputName,'gyrFeedbackModel/','');