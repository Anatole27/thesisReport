%% Find the optimal tuning
% raw feedback

% Get weights
weightsHinf;

% Inputs and outputs considered
w = inputs;
z = outputs;
u = {'\theta_{Tref}'};
y = {'a_{z83}';'q_{83}'};

n_meas = length(y);
n_con = length(u);

% Augmented system
missileW = blkdiag(Wout,eye(18))*missile([z;missile.y],[w;u])*blkdiag(Win,eye(n_con));

% Hinf unstructured design
[KHinf,~,GAM,INFO] = hinfsyn(missileW,18,length(u));
GAM

% Closed loop
allIn = missile.u;
allOut = missile.y;
KHinf = minreal(KHinf);
KHinf = slowfast(Khinf,5);
CLraw = lft(missile([allOut;missile.y],[allIn;u]),KHinf);