%% Bodes
%% azref/azerr
figure
hold on
sigma(CLnotch.NominalValue('a_{zerr}','a_{zref}'),'-k.');
sigma(CLstrain.NominalValue('a_{zerr}','a_{zref}'),'k--');
sigma(CLgyr.NominalValue('a_{zerr}','a_{zref}'),'k-o');
sigma(CLacc.NominalValue('a_{zerr}','a_{zref}'),'k-p');
h=legend('Notch','Strain .','Gyro','Acc')
xlim([0.1 1000])
set(h,'Location','SouthEast')
grid
print('./fig/sigmaErr', '-depsc'); %<-Save as PNG with 300 DPI

%% Vibration creation by rocket motor
figure
subplot(1,2,1)
hold on
w = logspace(1.8,2.4,80);
sigma(CLnotch.NominalValue('a_{z83v}','\theta_{Tpert}'),w,'-k.');
sigma(CLstrain.NominalValue('a_{z83v}','\theta_{Tpert}'),w,'k--');
sigma(CLgyr.NominalValue('a_{z83v}','\theta_{Tpert}'),w,'k-o');
sigma(CLacc.NominalValue('a_{z83v}','\theta_{Tpert}'),w,'k-p');
legend('Notch','Strain','Gyro','Acc')
grid

subplot(1,2,2)
hold on
w = logspace(1.8,2.4,80);
sigma(CLnotch.NominalValue('q_{83v}','\theta_{Tpert}'),w,'-k.');
sigma(CLstrain.NominalValue('q_{83v}','\theta_{Tpert}'),w,'k--');
sigma(CLgyr.NominalValue('q_{83v}','\theta_{Tpert}'),w,'k-o');
sigma(CLacc.NominalValue('q_{83v}','\theta_{Tpert}'),w,'k-p');
legend('Notch','Strain','Gyro','Acc')
grid
print('./fig/sigVibes', '-dpng', '-r300');

%% Stress creation by rocket motor
figure
hold on
w = logspace(1.8,2.4,80);
sigma(CLnotch.NominalValue('\epsilon_{46}','\theta_{Tpert}'),w,'-k.');
sigma(CLstrain.NominalValue('\epsilon_{46}','\theta_{Tpert}'),w,'k--');
sigma(CLgyr.NominalValue('\epsilon_{46}','\theta_{Tpert}'),w,'k-o');
sigma(CLacc.NominalValue('\epsilon_{46}','\theta_{Tpert}'),w,'k-p');
legend('Notch','Strain','Gyro','Acc')
grid
print('./fig/sigStress', '-dpng', '-r300');

%% theta demand for azref
close all
figure
subplot(1,2,1)
hold on
w = logspace(0,3,50);
sigma(1/Wout(2,2),w,'k');
sigma(CLnotch.NominalValue('\theta_T','a_{zref}'),w,'-k.');
sigma(CLstrain.NominalValue('\theta_T','a_{zref}'),w,'k--');
sigma(CLgyr.NominalValue('\theta_T','a_{zref}'),w,'k-o');
sigma(CLacc.NominalValue('\theta_T','a_{zref}'),w,'k-p');
% ylim([-120,-40])
h=legend('1/W_{\theta_T}','Notch','Strain','Gyro','Acc')
set(h,'location','northeast')
grid


subplot(1,2,2)
hold on
w = logspace(1.8,2.4,50);
sigma(1/Wout(2,2),w,'k');
sigma(CLnotch.NominalValue('\theta_T','a_{zref}'),w,'-k.');
sigma(CLstrain.NominalValue('\theta_T','a_{zref}'),w,'k--');
sigma(CLgyr.NominalValue('\theta_T','a_{zref}'),w,'k-o');
sigma(CLacc.NominalValue('\theta_T','a_{zref}'),w,'k-p');
% ylim([-120,-40])
h = legend('1/W_{\theta_T}','Notch','Strain','Gyro','Acc')
grid
print('./fig/sigThetaDemand', '-depsc');

%% rocket engine noise to actuator position (parasites)
figure
hold on
w = logspace(1.8,2.4,80);
sigma(CLnotch.NominalValue('\theta_T','\theta_{Tpert}'),w,'-k.');
sigma(CLstrain.NominalValue('\theta_T','\theta_{Tpert}'),w,'k--');
sigma(CLgyr.NominalValue('\theta_T','\theta_{Tpert}'),w,'k-o');
sigma(CLacc.NominalValue('\theta_T','\theta_{Tpert}'),w,'k-p');
h=legend('Notch','Strain','Gyro','Acc');
set(h,'Location','southeast')
grid
print('./fig/sigParasites', '-depsc');

%% Steps

figure
hold on
t = linspace(0,1.5,100);
step(CLnotch.NominalValue('a_{zerr}','a_{zref}'),t,'-k.');
step(CLstrain.NominalValue('a_{zerr}','a_{zref}'),t(end),'k--');
step(CLgyr.NominalValue('a_{zerr}','a_{zref}'),t,'k-o');
step(CLacc.NominalValue('a_{zerr}','a_{zref}'),t,'k-p');
legend('Notch','Strain','Gyro','Acc')
grid
% plotSetup(gca,[]);
print('./fig/step', '-depsc');

%% fins demand for azref
close all
figure
hold on
w = logspace(-1.3,3,50);
sigma(CLstrain.NominalValue('\delta_F','a_{zref}'),w,'k--');
sigma(CLgyr.NominalValue('\delta_F','a_{zref}'),w,'k-o');
sigma(CLacc.NominalValue('\delta_F','a_{zref}'),w,'k-p');
h=legend('Strain','Gyro','Acc')
% set(h,'location','south')
grid
print('./fig/sigDeltaDemand', '-depsc');