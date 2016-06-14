% Two partite rocket.

% Init
m = m1+m2;
I = I1 + I2 + m1*(m2/(m1+m2)*(l1/2+l2/2))^2 + m2*(m1/(m1+m2)*(l1/2+l2/2))^2;

M1 = I;
D1 = 0;
K1 = CLa2*(m1/(m1+m2)*(l1/2+l2/2)) - CLa1*(m2/(m1+m2)*(l1/2+l2/2));
CpCq = [m1/(m1+m2)*(l1/2+l2/2)+l2/2 -(m2/(m1+m2)*(l1/2+l2/2)+l2/2)];

% Equations
A1 = [0 1;
    -inv(M1)*K1 -inv(M1)*D1];
A1 = A1-1/100000*eye(2);
B1 = [0 0; inv(M1)*CpCq];

% Augmenting system
C1 = [eye(2);zeros(1,2);eye(2)]; D1 = [zeros(2,2);0 1;zeros(2,2)];
G1 = ss(A1,B1,C1,D1);

% Actuator dynamics
s = zpk('s');
wc = 200;
w_act = 1/(s/wc+1);
clear Win
Win(2,2) = w_act;
Win(1,1) = 1;

G1 = G1*Win;

% Weights
wc = 100;
wmax = 10^4;
wmin = 10^-3;
w1 = 1000/(2*pi/180)*(wc+s)/(s+wmin);
w2 = 1000/5*(wc+s)/(s+wmin);
w3 = (1+s/wc)/(1+s/wmax);
clear Wout
Wout(1,1) = w1*eye(1);
Wout(2,2) = w2*eye(1);
Wout(3:5,3:5) = eye(3);

Gw = Wout*G1;

% Control
[K1,CL1,GAM,INFO] = hinfsyn(Gw([1 2 3 4 5],:),2,1);
Kslow = freqsep(K1,100);
CL1 = lft(G1,K1);
% 
% subplot(3,1,1)
% sigma(CL1(1,1));
% hold on
% sigma(zpk(1/w1));
% 
% subplot(3,1,2)
% sigma(CL1(2,1));
% hold on
% sigma(zpk(1/w2));
% 
% subplot(3,1,3)
% sigma(CL1(3,1));
% hold on
% sigma(zpk(1/w3));
% 
% figure
% pzmap(CL1)