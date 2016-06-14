actuatorPlacement_21_05
%% Augmenting the system

% Actuators
i_thruster = 1;
i_PIF = floor(3/l)+1;
Bw = B(:,[i_thruster]);
Bu = B(:,actuatorList);
[~,pw] = size(Bw);
[~,pu] = size(Bu);
p = pw + pu;
B = [Bw, Bu];

% Sensors
i_IMU = floor(4.5/l)+1;
Cy = C(sensorList,:);
[ry,~] = size(Cy);
Cz = [Vm; Cy; zeros(pu,2*n_modes); Phi(i_IMU,:)*Vm(1:n_modes,:)];
[rz,~] = size(Cz);
C = [Cz; Cy];
r = ry+rz;

D = [zeros(rz-pu-1,p); [zeros(pu,pw), eye(pu)]; zeros(ry+1,p)];

G_OL = ss(A,B,C,D);

%% Weights
% In (Normalizing)
% Win(1,1) = zpk(26000);
% for i = 1:pu
% Win(i+pw,i+pw) = zpk(1);
% end

% Out
s = zpk('s');
wc = 100*2*pi;
wmax = 10^4;
wmin = 10^-3;
k = abs(evalfr(G_OL(1,1),0));
zeta = 0.5;
clear Wqm
% Wqm(1,1) = 1/k*(1+2*zeta/om(1)*s+(s/om(1))^2);
% for i = 2:n_modes
% Wqm(i,i) = 1/k*(1+2*z(i)/om(i)*s+(s/om(i))^2);
% end
for i = 1:n_modes
    Wqm(i,i) = zpk(1/(2*k));
end

WqF = (1+s/wc)^2/(1+s/wmax)^2;
WqIMU = 10^(120/20);

clear Wout
Wout(1:n_modes,1:n_modes) = Wqm;
Wout(rz-pu:rz-1,rz-pu:rz-1) = WqF*eye(pu);
Wout(rz,rz) = WqIMU;
Wout(rz+1:r,rz+1:r) = eye(ry);
%Wout(5,5) = w3;
%Wout(6:9,6:9) = eye(4);
G_OLw = Wout*G_OL;



%% Control
n_meas = length(sensorList);
n_con = length(actuatorList);
[K,G_CLw,GAM,INFO] = hinfsyn(G_OLw([1,rz-pu:rz-1,r-ry+1:r],:),n_meas,n_con);
G_CL = lft(G_OL,K);

%% Sigma check
close all
hold on
sigma(zpk(1/Wqm(1,1)),'r');
for i = 1:n_modes
    sigma(G_OL(i,1),'-.')
    sigma(G_CL(i,1),'.')
end
legend('Upper Limit','Open-Loop','Closed-Loop');
title('Modes amplitude')
hold off

figure
sigma(1/WqF(1,1),'r');
hold on
for i = rz-pu:rz-1
    sigma(G_CL(i,1),'b');
end
legend('Upper Limit','System singular value');
title('$|F_{control}/F_{disturbances}|$','interpreter','latex');
