% Two partite rocket.

% Init
m1 = 310;
m2 = 140;
l1 = 2.2;
l2 = 2.7;
I1 = m1*l1^2/4;
I2 = m2*l2^2/4;

Ddart = 0.18;
Dboost = 0.36;
d = 0.14; % 2cm of thickness
E = 210*10^9;
l_bend = 0.5; % link = 50cm
Iy = pi/64*(Ddart^4-d^4);
k = 1; % Gimbal link
lambda = k/1000;

S = pi/4*Ddart^2;
q = 1/2*1.22*(4.5*340)^2;
CLa1 = 2*pi*Dboost^2/Ddart^2;
CLa2 = 2*pi;

M = [m1 m2 0 0;
    -l1/2*m1 0 I1 0;
    -l2/2*m1 0 0 I2;
    1 -1 l1/2 l2/2];
D = zeros(4,2);
D(2:3,:) = lambda*[1 -1; -1 1];
K = [-q*CLa1*S -q*CLa2*S;
    l1/2*q*CLa1*S+k -k;
    l2/2*q*CLa1*S-k  k;
    0 0];
CpCq = [1 1; 0 -l1; l2/2 -l2/2; 0 0];

% Equations
red = [zeros(2) eye(2)];
A = [zeros(2) eye(2);
    -red*inv(M)*K -red*inv(M)*D];
A = A+1/100000*eye(4);
B = [zeros(2); red*inv(M)*CpCq];

% Augmenting system
C = [eye(4);zeros(1,4);eye(4)]; D = [zeros(4,2);0 1;zeros(4,2)];
G = ss(A,B,C,D);

% Actuator dynamics
clear Win
s = zpk('s');
wc = 200;
w_act = 1/(s/wc+1);
Win(2,2) = w_act;
Win(1,1) = 1;

G = G*Win;

% Weights
clear Wout
wc = 100;
wmax = 10^4;
wmin = 10^-3;
w1 = 1000/(2*pi/180)*(wc+s)/(s+wmin);
w2 = 1000/5*(wc+s)/(s+wmin);
w3 = (1+s/wc)/(1+s/wmax);
Wout(1:2,1:2) = w1*eye(2);
Wout(3:4,3:4) = w2*eye(2);
Wout(5,5) = w3;
Wout(6:9,6:9) = eye(4);

Gw = Wout*G;

% Control
[K,CL,GAM,INFO] = hinfsyn(Gw,4,1);
%Kslow = freqsep(K,100);
CL = lft(G,K);

% subplot(3,1,1)
% sigma(zpk(1/w1));
% hold on
% sigma(CL(1,1));
% sigma(CL(2,1));
% 
% subplot(3,1,2)
% sigma(zpk(1/w2));
% hold on
% sigma(CL(3,1));
% sigma(CL(4,1));
% 
% subplot(3,1,3)
% sigma(zpk(1/w3));
% hold on
% sigma(CL(5,1));
% 
% figure
% pzmap(CL)