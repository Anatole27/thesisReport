%% System creation
n = 4;
r = 2;
m = 2;
sys = rss(n,m,r);

%% Simulation
%Time
t_end = 30;
dt = 0.05; % Time increment
time = 0:dt:t_end; % Time vector
l = length(time); % Number of samples

%Input
input = wgn(l,r,0);

%Output
output = lsim(sys,input,time);
