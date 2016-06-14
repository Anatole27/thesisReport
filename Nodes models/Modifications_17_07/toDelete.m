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

p1.LineStyle = 'none'
p2.LineStyle = 'none'
p1.Color = 'k';
p2.Color = 'k';
p1.Marker = 'x'
p2.Marker = 'o'
grid minor
xlabel('Node or beam number $i$','Interpreter','latex')
ylabel(ax(1),'Mass of nodes $m_i$ ($kg$)','Interpreter','latex')
ylabel(ax(2),'Bending stiffness $E_iI_i$ ($N.m^2$)','Interpreter','latex')
legend('Mass of nodes','Beam bending stiffness')
xlim(ax(1),[0,21])
xlim(ax(2),[0,21])
plotSetup(ax(1),[])
plotSetup(ax(2),[])
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/beamStructDimGraph', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

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
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/fqConvergence', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

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
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/modeShape', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%%
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/hsvdAcc', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

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
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesEps', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

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
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesGyr', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

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
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/indicesAcc', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Root Locus without vibe damping
dur = ss(Arb,Brb,Crb,Drb); dur.y = names_rb.y;
rlocus(-flexiMissile('q_{83}',1),'k');
xmin = -120; xmax = 20; ymin = -200; ymax = 200;
axis([xmin xmax ymin ymax])
plotSetup(gca,[]);
grid off
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusNoVibeSupp', '-dpng', '-r300'); %<-Save as PNG with 300 DPI


%% Notch filtering
bode(notchFilter,logspace(1,3,1000),'k');
plotSetup(gca,[]);
%% Bode after filtering
points = logspace(0,3,1000);
bode(flexiMissile('q_{83}',1),points,'k')
hold on
bode(filteredMissile('q_{83}',1),points,'k--')
yolo = gca; yolo.Title.String = '';
legend('Original system','Filtered system')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeFiltered', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Bode thrust vect
s=zpk('s');
bode(1/(s^2/wT^2+2*zT/wT*s + 1),'k');
plotSetup(gca,[])
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeThrustVect', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Bode fins
s=zpk('s');
bode(1/(s^2/wF^2+2*zF/wF*s + 1),'k');
plotSetup(gca,[])
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeFins', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Fins 2 Strain TF
[Afs,Bfs,Cfs,Dfs] = linmod('onlyFins2Strain');
f2s = tf(ss(Afs,Bfs,Cfs,Dfs));
[num,den] = tfdata(f2s);
syms s
t_sym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
digits(3)
latex(vpa(t_sym))

%% Strain root locus
rlocus(-flexiMissile('\epsilon_{46}','\delta_{Fref}'),'k');
hline = findobj(gcf, 'type', 'line');
for i = 1:length(hline)
    hline(i).LineWidth = 1;
end
axis([-250 50 -1000 1000]);
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusStrain', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Strain FB Bode
points = logspace(0,3,1000);
bode(flexiMissile('q_{83}',1),'k',loopEps('q_{83}',1),points,'k--');
legend('No Damping','Active Damping')
plotSetup(gca,[]);
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingStrain', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% rlocus gyr diff
rlocus(flexiMissile('\Delta_q','\delta_{Fref}'),'k')
hline = findobj(gcf, 'type', 'line');
for i = 1:length(hline)
    hline(i).LineWidth = 1;
end
axis([-250 50 -1000 1000]);
grid minor
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusDGyr', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Bode Gyr feedback
points = logspace(0,3,1000);
bode(flexiMissile('q_{83}',1),'k',loopD('q_{83}',1),points,'k--');
legend('No Damping','Active Damping')
plotSetup(gca,[]);
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingGyr', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% rlocus Acc feedback with integrator
s = zpk('s');
rlocus(1/s*flexiMissile('\Sum a_z','\delta_{Fref}'),'k')
hline = findobj(gcf, 'type', 'line');
for i = 1:length(hline)
    hline(i).LineWidth = 1;
end
axis([-250 50 -1000 1000]);
grid minor
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/rlocusSAcc', '-dpng', '-r300'); %<-Save as PNG with 300 DPI

%% Bode Acc feedback

points = logspace(0,3,1000);
bode(flexiMissile('q_{83}',1),'k',loopA('q_{83}',1),points,'k--');
legend('No Damping','Active Damping')
plotSetup(gca,[]);
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/bodeDampingAcc', '-dpng', '-r300'); %<-Save as PNG with 300 DPI