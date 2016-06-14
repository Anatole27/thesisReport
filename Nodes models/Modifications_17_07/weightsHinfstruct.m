%% This script creates weights for Hinfstruct
s = zpk('s');
addPole = 1/(s/10000 + 1);

%% Naming
inputs = {};
outputs = {};
Win = [];
Wout = [];

%% Outputs

% az_err
wAccMax = 5;
WErr = 1/(  s/wAccMax  );
Wout = blkdiag(Wout,WErr);
outputs = [outputs; 'a_{zerr}'];

% Thrust vectoring orientation
WTheta = 1/( 0.5*0.01*wT^2/s^2  )*addPole^2;
Wout = blkdiag(Wout,WTheta);
outputs = [outputs; '\theta_T'];

% Fins deflection
% WFins = 1/(  0.01*wF/s^2  )*addPole^2;
% Wout = blkdiag(Wout,WFins);
% outputs = [outputs; '\delta_F'];

% Sensor pack vibrations

% Acc
% avMax = 0.015;
% Wav = 1/(  avMax  );
% Wout = blkdiag(Wout,Wav);
% outputs = [outputs; 'a_{z83v}'];

% % Gyr
% qvMax = 0.01;
% Wqv = 1/(  qvMax  );
% Wout = blkdiag(Wout,Wqv);
% outputs = [outputs; 'q_{83v}'];



%% Inputs

% az reference
Win = blkdiag(Win,1);
inputs = [inputs; 'a_{zref}'];

% % sensors noise
% % gyro
% wN = 50;
% Qn = 0.001;
% Wqn = Qn*s/(s + wN);
% Win = blkdiag(Win,Wqn,Wqn);
% inputs = [inputs; 'q_{10n}'; 'q_{83n}'];
% 
% % acc
% An = 0.01;
% Wan =  An*s/(s + wN);
% Win = blkdiag(Win,Wan,Wan,Wan,Wan);
% inputs = [inputs; 'a_{z10n}'; 'a_{z53n}'; 'a_{z83n}'; 'a_{z92n}'];

% Thrust misalignment
% Tmis = 3*10^-6;
% WTmis = Tmis;
% Win = blkdiag(Win,WTmis);
% inputs = [inputs; '\theta_{Tpert}'];
