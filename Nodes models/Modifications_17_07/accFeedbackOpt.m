%% Find the optimal tuning
% acc feedback

% Get weights
weightsHinfstruct;

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
load_system('accFeedbackModel')
sys = ulinearize('accFeedbackModel');
CLacc = ss(sys.a,sys.b,sys.c,sys.d);
CLacc.u = strrep(sys.InputName,'accFeedbackModel/','');
CLacc.y = strrep(sys.OutputName,'accFeedbackModel/','');
CLacc = minreal(CLacc);

% Assessement
% close all
% bode(CLacc('a_{zerr}','a_{zref}'),1/WErr);
% figure
% bode(CLacc('\theta_T','a_{zref}'),1/WTheta);
% figure
% step(CLacc('a_{zerr}','a_{zref}'));