%% Find the optimal tuning
% raw feedback

% Get weights
weightsHinfstruct;

% Init gains
Kq = 0.0345;
Ki = 0.0033;
Kp = 6.6903e-06;

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('rawFeedbackModel',{'PI','Kq'});

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
load_system('rawFeedbackModel')
sys = ulinearize('rawFeedbackModel');
CLraw = ss(sys.a,sys.b,sys.c,sys.d);
CLraw.u = strrep(sys.InputName,'rawFeedbackModel/','');
CLraw.y = strrep(sys.OutputName,'rawFeedbackModel/','');

% Assessement
% close all
% bode(CLraw('a_{zerr}','a_{zref}'),1/WErr);
% figure
% bode(CLraw('\theta_T','a_{zref}'),1/WTheta);
% figure
% step(CLraw('a_{zerr}','a_{zref}'));