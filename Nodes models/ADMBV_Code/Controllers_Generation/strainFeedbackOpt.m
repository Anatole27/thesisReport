%% Finds the optimal tuning for the strain architecture
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
% * CLstrain: closed-loop of the strain architecture

% Init gains
Keps = 600;
Kq = 0.0375;
Ki = 0.035;
Kp = 2.3626e-05;

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('strainFeedbackModel',{'PI','Kq'});

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
load_system('strainFeedbackModel')
sys = ulinearize('strainFeedbackModel');
CLstrain = ss(sys.a,sys.b,sys.c,sys.d);
CLstrain.u = strrep(sys.InputName,'strainFeedbackModel/','');
CLstrain.y = strrep(sys.OutputName,'strainFeedbackModel/','');