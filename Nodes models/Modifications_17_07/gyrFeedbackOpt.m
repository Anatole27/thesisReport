%% Find the optimal tuning
% gyr feedback

% Get weights
weightsHinfstruct;

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
rng(0)
opt = hinfstructOptions('Display','final','RandomStart',5);
weightedCL = hinfstruct(Wout*tunable*Win,opt);

% Get tunables
Kq = getBlockValue(weightedCL,'Kq');
Kq = Kq.d;
PI = getBlockValue(weightedCL,'PI');
Kp = PI.Kp;
Ki = PI.Ki;

% Get tuned system
load_system('gyrFeedbackModel')
sys = ulinearize('gyrFeedbackModel');
CLgyr = ss(sys.a,sys.b,sys.c,sys.d);
CLgyr.u = strrep(sys.InputName,'gyrFeedbackModel/','');
CLgyr.y = strrep(sys.OutputName,'gyrFeedbackModel/','');

% Assessement
% close all
% bode(CLgyr('a_{zerr}','a_{zref}'),1/WErr);
% figure
% bode(CLgyr('\theta_T','a_{zref}'),1/WTheta);
% figure
% step(CLgyr('a_{zerr}','a_{zref}'));