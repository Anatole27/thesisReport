% Script to generate figures
addpath('/home/anatole/Documents/Etudes/Cranfield University/Thesis/Nodes models/Oscillation_control_12_07')

set(0,'DefaultTextFontName', 'LM Roman 12')
set(0,'DefaultTextFontSize',20)
set(0,'DefaultAxesFontName', 'LM Roman 12')
set(0,'DefaultAxesFontSize',20)
set(0,'DefaultAxesXLabelFontSize', 20)
return
%% Beam dimensions
% init model
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xAE,xCG,Jy,xIMU,CD0,kD,Dref,Vx,gamma,acc,X0,Z0] = loadModel_EB(20)
node = 1:n;
[ax,p1,p2] = plotyy(node,dm,node(1:n-1),EI);

set(p1,'LineStyle','none','Color' , 'k','Marker', 'x','LineWidth',2,'MarkerSize',10);
set(p2,'LineStyle','none','Color' , 'k','Marker', 'o','LineWidth',2)
grid minor
x = xlabel('Node or beam number $i$','Interpreter','latex')
y1 = ylabel(ax(1),'Mass of nodes $m_i$ ($kg$)','Interpreter','latex')
y2 = ylabel(ax(2),'Bending stiffness $E_iI_i$ ($N.m^2$)','Interpreter','latex')
set([y1 y2 x],'FontSize',20)
h = legend('Mass of nodes','Beam bending stiffness')
set(h,'Location','East')
xlim(ax(1),[0,21])
xlim(ax(2),[0,21])
plotSetup(ax(1),[])
plotSetup(ax(2),[])
print('./fig/beamStructDimGraph', '-depsc'); %<-Save as PNG with 300 DPI

%% Frequency convergence

N = 300;
om_rec = zeros(5,N);
N = 7:300;
for n = 7:300
    n
    [EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xAE,xCG,Jy,xIMU,CD0,kD,Dref,Vx,gamma,acc,X0,Z0] = loadModel_EB(n);
    
    % Get 2nd order structural model
    [M,D,K,Bo,Coz,Cov,Do] = getMDK_Leckie(EI,l,n,dm,Diam);
    
    % Put into modal representation without rigid-body modes (X = [qm,qm_dot])
    % High order modes are truncated
    n_modes = 5; % number of structural modes kept
    [Mm,Dm,Km,Bm,Cmz,Cmv,Do,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coz,Cov,Do,n_modes,m,xCG,Jy,l,n);
    om_rec(:,n) = om;
end
%%
figure; hold on;
scatter(N',om_rec(1,N)'/(2*pi),'xk');
plotSetup(gca,[]);
xlabel('Number of nodes $n$','Interpreter','latex')
ylabel('First bending mode frequency $\omega_1$ ($Hz$)','Interpreter','latex')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/fqConvergence', '-depsc'); %<-Save as PNG with 300 DPI

%% Modes shape
[EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xAE,xCG,Jy,xIMU,CD0,kD,Dref,Vx,gamma,acc,X0,Z0] = loadModel_EB(100);

% Get 2nd order structural model
[M,D,K,Bo,Coz,Cov,Do] = getMDK_Leckie(EI,l,n,dm,Diam);

% Put into modal representation without rigid-body modes (X = [qm,qm_dot])
% High order modes are truncated
n_modes = 5; % number of structural modes kept
[Mm,Dm,Km,Bm,Cmz,Cmv,Do,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coz,Cov,Do,n_modes,m,xCG,Jy,l,n);


% Add rb modes
Phi01 = 0.1*ones(100,1);
Phi02 = linspace(-0.1,0.1,100)';
Phi = [Phi01 Phi02 Phi];

% Plotting
x = linspace(0,4.9,100)';
yolo = plot(x,Phi(:,1:5));
set(gca,'YDir','reverse');
xlabel('Abscissa $x$ ($m$)','interpreter','latex');
ylabel('Displacement $z$ ($m$)','interpreter','latex');
ylim([-0.28,0.15]);
h = legend('Translation mode','Rotation mode','$1^{st}$ bending mode','$2^{nd}$ bending mode','$3^{rd}$ bending mode','Location','northwest');
plotSetup(gca,yolo);
set(h,'interpreter','latex')
set(h,'location','northwest')


% Save
%print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/modeShape', '-depsc'); %<-Save as PNG with 300 DPI

%%
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/hsvdAcc', '-depsc'); %<-Save as PNG with 300 DPI

%% Placement
[A,B,C,D] = ssdata(flexi(1:3*n-2,[1 40]));

%% Strain gages
sigEps = zeros(n,1);
sigEps2 = zeros(n,1);
smin = 46;
smax = 89;
for i=2:n-1
    if i>= smin && i<= smax
        sigEps(i) = norm(B(1:2,:))*norm(C(i-1,1:2))/(z(1)*om(1));
    else
        sigEps2(i) = norm(B(1:2,:))*norm(C(i-1,1:2))/(z(1)*om(1));
    end
    i-1
end
[~,imax] = max(sigEps2);
sigEps2(imax)=2*sigEps(imax+1)-sigEps(imax+2);

epsPlot = bar([sigEps,sigEps2]);
xlabel('Node number $i$','interpreter','latex');
ylabel('Placement index $\sigma_i$','interpreter','latex')
plotSetupBar
xlim([2,99])
hbar = findobj(gcf, 'type', 'bar');
hbar(1).EdgeColor = [0.5 0.5 0.5];
hbar(2).EdgeColor = [0 0 0];
hbar(1).FaceColor = [0.5 0.5 0.5];
hbar(2).FaceColor = [0 0 0];
legend('Possible location','Impossible location')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesEps', '-depsc'); %<-Save as PNG with 300 DPI

%% Gyros
sigGyr = zeros(n,1);
sigGyr2 = zeros(n,1);
smin = 10;
smax = 92;
for i=1:n
    if i>= smin && i<= smax
        sigGyr(i) = norm(B(1:2,:))*norm(C(n-2+i,1:2))/(z(1)*om(1));
    else
        sigGyr2(i) = norm(B(1:2,:))*norm(C(n-2+i,1:2))/(z(1)*om(1));
    end
    i+n-2
end

epsPlot = bar([sigGyr,sigGyr2]);
xlabel('Node number $i$','interpreter','latex');
ylabel('Placement index $\sigma_i$','interpreter','latex')
plotSetupBar
xlim([1,100])
hbar = findobj(gcf, 'type', 'bar');
hbar(1).EdgeColor = [0.5 0.5 0.5];
hbar(2).EdgeColor = [0 0 0];
hbar(1).FaceColor = [0.5 0.5 0.5];
hbar(2).FaceColor = [0 0 0];
legend('Possible location','Impossible location')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesGyr', '-depsc'); %<-Save as PNG with 300 DPI

%% Accelerometers
sigAcc = zeros(n,1);
sigAcc2 = zeros(n,1);
smin = 10;
smax = 92;
for i=1:n
    if i>= smin && i<= smax
        sigAcc(i) = norm(B(1:2,:))*norm(C(2*n-2+i,1:2))/(z(1)*om(1));
    else
        sigAcc2(i) = norm(B(1:2,:))*norm(C(2*n-2+i,1:2))/(z(1)*om(1));
    end
end

bar([sigAcc,sigAcc2]);
xlabel('Node number $i$','interpreter','latex');
ylabel('Placement index $\sigma_i$','interpreter','latex')
plotSetupBar
xlim([1,100])
hbar = findobj(gcf, 'type', 'bar');
hbar(1).EdgeColor = [0.5 0.5 0.5];
hbar(2).EdgeColor = [0 0 0];
hbar(1).FaceColor = [0.5 0.5 0.5];
hbar(2).FaceColor = [0 0 0];
legend('Possible location','Impossible location')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesAcc', '-depsc'); %<-Save as PNG with 300 DPI

%% Root Locus without vibe damping
dur = ss(Arb,Brb,Crb,Drb); dur.y = names_rb.y;
rlocus(-flexiMissile('q_{83}',1),'k');
xmin = -120; xmax = 20; ymin = -200; ymax = 200;
axis([xmin xmax ymin ymax])
plotSetup(gca,[]);
grid off
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusNoVibeSupp', '-depsc'); %<-Save as PNG with 300 DPI


%% Notch filtering
bode(notchFilter,logspace(1,3,1000),'k');
grid
print('./fig/notchFilter', '-depsc'); %<-Save as PNG with 300 DPI

%% 


%% Bode after filtering
points = logspace(0,3,1000);
bode(missile.NominalValue('q_{83}','\theta_{Tref}'),points,'k')
hold on
bode(filteredMissile.NominalValue('q_{83}','\theta_{Tref}'),points,'k--')
h=legend('Original system','Filtered system');
set(h,'location','southwest');
grid
print('./fig/bodeFiltered', '-depsc'); %<-Save as PNG with 300 DPI

%% Bode thrust vect
s=zpk('s');
bode(1/(s^2/wT^2+2*zT/wT*s + 1),'k');
grid
h = legend('Nozzles actuator');
set(h,'interpreter','latex','fontsize',12)
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeThrustVect', '-depsc'); %<-Save as PNG with 300 DPI

%% Bode fins
s=zpk('s');
bode(1/(s^2/wF^2+2*zF/wF*s + 1),'k');
%plotSetup(gca,[])
grid
h = legend('Fins actuator');
set(h,'interpreter','latex','fontsize',12)
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeFins', '-depsc'); %<-Save as PNG with 300 DPI

%% Fins 2 Strain TF
[Afs,Bfs,Cfs,Dfs] = linmod('onlyFins2Strain');
f2s = tf(ss(Afs,Bfs,Cfs,Dfs));
[num,den] = tfdata(f2s);
syms s
t_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
digits(3)
latex(vpa(t_sym))

f2s = tf(notchFilter);
[num,den] = tfdata(f2s);
syms s
t_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
digits(3)
latex(vpa(t_sym))

%% Strain root locus
rlocus(-missile.NominalValue('\epsilon_{46}','\delta_{Fref}'),'k');
pList = pole(missile);
zList = tzero(missile.NominalValue('\epsilon_{46}','\delta_{Fref}'));
hold on
scatter(real(pList),imag(pList),'xk','linewidth',1);
scatter(real(zList),imag(zList),'ok','linewidth',1);
legend('Root Locus','Poles','Zeros')
grid
axis([-250 50 -1000 1000]);
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusStrain', '-dpng'); %<-Save as PNG with 300 DPI

%% Strain FB Bode
points = logspace(0,3,1000);
bode(missile.NominalValue('q_{83}','\theta_{Tref}'),'k',loopEps.NominalValue('q_{83}','\theta_{Tref}'),points,'k--');
legend('No Damping','Active Damping')
% print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingStrain', '-depsc'); %<-Save as PNG with 300 DPI

%% rlocus gyr diff
rlocus(missile.NominalValue('\Delta_q','\delta_{Fref}'),'k')
pList = pole(missile);
zList = tzero(missile.NominalValue('\Delta_q','\delta_{Fref}'));
hold on
scatter(real(pList),imag(pList),'xk','linewidth',1);
scatter(real(zList),imag(zList),'ok','linewidth',1);
legend('Root Locus','Poles','Zeros')
axis([-250 50 -1000 1000]);
grid minor
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusDGyr', '-dpng'); %<-Save as PNG with 300 DPI

%% Bode Gyr feedback
points = logspace(0,3,1000);
bode(missile.NominalValue('q_{83}','\theta_{Tref}'),'k',loopD.NominalValue('q_{83}','\theta_{Tref}'),points,'k--');
legend('No Damping','Active Damping')
% plotSetup(gca,[]);
% print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingGyr', '-depsc'); %<-Save as PNG with 300 DPI

%% rlocus Acc feedback with integrator
s = zpk('s');
rlocus(1/s*missile.NominalValue('\Sum a_z','\delta_{Fref}'),'k')
pList = pole(missile);
zList = tzero(missile.NominalValue('\Sum a_z','\delta_{Fref}'));
hold on
scatter(real(pList),imag(pList),'xk','linewidth',1);
scatter(real(zList),imag(zList),'ok','linewidth',1);
legend('Root Locus','Poles','Zeros')
axis([-250 50 -1000 1000]);
grid minor
%print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusSAcc', '-dpng'); %<-Save as PNG with 300 DPI

%% Bode Acc feedback

points = logspace(0,3,1000);
bode(missile.NominalValue('q_{83}','\theta_{Tref}'),'k',loopA.NominalValue('q_{83}','\theta_{Tref}'),points,'k--');
legend('No Damping','Active Damping')
% plotSetup(gca,[]);
% print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingAcc', '-depsc'); %<-Save as PNG with 300 DPI

%% rlocus pitch rate
rigid = ss(Arb,Brb,Crb,Drb);
rigid.y = names_rb.y;
rigid.u = names_rb.u;
rlocus(-rigid.NominalValue('q','\theta_T'),'k');
hline = findobj(gcf, 'type', 'line');
for i = 1:length(hline)
    hline(i).LineWidth = 1;
end
axis([-250 50 -1000 1000]);
grid minor

%% pzmap uncerttainty
limts = [-8,1,-160,160]
pzmapZoom(CLraw,limts)
print('./fig/pzRaw', '-depsc');

pzmapZoom(CLnotch,limts)
print('./fig/pznotch', '-depsc'); %<-Save as PNG with 300 DPI

pzmapZoom(CLstrain,limts)
print('./fig/pzstrain', '-depsc'); %<-Save as PNG with 300 DPI

pzmapZoom(CLgyr,limts)
print('./fig/pzgyr', '-depsc'); %<-Save as PNG with 300 DPI

pzmapZoom(CLacc,limts)
print('./fig/pzacc', '-depsc'); %<-Save as PNG with 300 DPI

%%
opt = stepDataOptions('StepAmplitude',pi/180);
step(missile.NominalValue('a_{z83}','\theta_{Tref}'),'k',10,opt);
title('')

%% Sketch simulations
close all
time = 0:0.00001:0.2; time = time';
n = length(time);
[ny,nu] = size(CLnotch);
inputs = zeros(n,nu);
inputs(:,1) = sin(time*2*pi/1);
inputs(:,2) = 0.00002*sin(time*2*pi*20)+0.0002*sin(time*268)+0.00001*sin(time*431);
outputs = lsim(CLnotch,inputs,time);
plot(outputs(:,end),-time,'k','LineWidth',2)
set(gca,'YTick',[]);set(gca,'XTick',[]);
set(gca, 'box','off')

print('./fig/simSketch', '-depsc'); %<-Save as PNG with 300 DPI

%%
close all
dur = ss(Arb,Brb,Crb,Drb); dur.u = names_rb.u; dur.y = names_rb.y;
rlocus(-dur.NominalValue('q','\theta_T'),'k')

print('./fig/rlocusKq', '-depsc'); %<-Save as PNG with 300 DPI

loopQ = feedback(dur,-0.074,1,2);
sisotool(loopQ('a_{z83}','\theta_T'));

%% Weights
subplot(1,2,1)
sigma(1/WErr,'k')
hline = findobj(gcf, 'type', 'line');
set(hline,'linewidth',1)
h=title('Singular Values of $\frac{1}{W_{err}}(s)$')
set(h,'interpreter','latex')
grid

subplot(1,2,2)
sigma(1/WTheta,'k')
hline = findobj(gcf, 'type', 'line');
set(hline,'linewidth',1)
h=title('Singular Values of $\frac{1}{W_{\theta_T}}(s)$')
set(h,'interpreter','latex')
grid
print('./fig/weights', '-depsc'); %<-Save as PNG with 300 DPI
