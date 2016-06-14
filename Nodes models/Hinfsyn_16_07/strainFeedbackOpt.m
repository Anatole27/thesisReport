%% Find the optimal tuning
% strain feedback

% Get weights
weightsHinfstruct;

% Init gains
Keps = 600;
Kq = 0.0375;
Ki = 0.035;
Kp = 2.3626e-05;

% Get Model
load_system('flexibleMissileFullIO')
ST0 = slTuner('strainFeedbackModel',{'PI','Kq'});

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
load_system('strainFeedbackModel')
sys = ulinearize('strainFeedbackModel');
CLstrain = ss(sys.a,sys.b,sys.c,sys.d);
CLstrain.u = strrep(sys.InputName,'strainFeedbackModel/','');
CLstrain.y = strrep(sys.OutputName,'strainFeedbackModel/','');

% Assessement
% close all
% bode(CLstrain('a_{zerr}','a_{zref}'),1/WErr);
% figure
% bode(CLstrain('\theta_T','a_{zref}'),1/WTheta);
% figure
% step(CLstrain('a_{zerr}','a_{zref}'));