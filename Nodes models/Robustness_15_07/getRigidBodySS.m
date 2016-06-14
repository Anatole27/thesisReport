function [Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names] = getRigidBodySS(S,rho,CL0,CLa,CLd,xFins,xAE,Cm0,Cma,Cmd,xIMU,CD0,kD,Dref,V0,X0,Z0,gamm0,Vdot0,l,dm,listAcc,Jy,xCG)

%% Computes SS matrices for rigib body system at x
% Author : Anatole VERHAEGEN
% INPUTS
% qS : dynamic pressure x reference surface (N)
% CL0, CLa : lift coefficients (zero value and slope) (alph in rad !!)
% CLd : lift coeff slope of ailerons
% xFins : fins position
% Cm0, Cma : pitching moment coefficients (zero value and slope)
% CD0, kD : zero lift drag and induced drag coefficient
% Dref : length reference (booster diameter) (m)
% Vx : Missile speed (m/s)
% gamm0 : flight path (rad)
% l : beam element length (m)
% dm : vector of nodes mass (kg) [nx1]

%OUTPUTS
% (Arb,Brb). Matrices of SS system of X = [y,y_dot,theta,theta_dot]

% Mass properties
m = sum(dm); % Mass
grav = 9.81;

% Lift, Moment and Drag coefficients
syms u gamm alph q thetaT deltaF T
CL = CL0 + alph*CLa + deltaF*CLd;
Cm = Cm0 + alph*Cma + deltaF*Cmd;
CD = CD0 + kD*CL^2;
V = V0*(u+1);
qS = 1/2*rho*S*V^2;

%% State Equations
% Nominal value of uncertainties

Vdot = -Vdot0 + 1/m*(-qS*CD + T*cos(alph+thetaT) - m*grav*sin(gamm));
gammdot = 1/(m*V)*(qS*CL + T*sin(alph+thetaT) - m*grav*cos(gamm));
alphdot = q - gammdot;
qdot = 1/Jy*(qS*Dref*Cm - T*sin(thetaT)*xCG);
system = [Vdot,gammdot,alphdot,qdot]';
systemEq = subs(system([1,2,4]),{gamm,u,q,deltaF},{gamm0,0,0,0});

% Approximate solution (gamm0 = 0)
alph0  = (m*grav/(1/2*rho*S*V0^2) -CL0)/CLa;
T0 = (1/2*rho*S*V0^2)*(CD0 + kD*(CL0 + CLa*alph0)^2) + Vdot0*m;
thetaT0 = 1/(T0*xCG)*(1/2*rho*S*V0^2)*(Dref*(Cm0+Cma*alph0));

% Solving
[T0,thetaT0,alph0] = vpasolve(systemEq,[ T, thetaT, alph],[T0,thetaT0,alph0]);
alph0 = double(alph0);
T0 = double(T0);
thetaT0 = double(thetaT0);

%% Compute SS system
[xCG,T0,Cma,Cmd] = addUncertaintyRB(xCG,T0,CLa,CLd,xFins,xAE,Dref);

A11 = -1/(m*V0)*(1/2*rho*S*V0^2*CLa + T0*cos(thetaT0 + alph0));
A12 = 1;
A21 = 1/Jy*1/2*rho*S*V0^2*Dref*Cma;
A22 = 0;
Arb = [A11 A12; A21 A22];

B1 =  [-T0*cos(thetaT0 + alph0),  -1/2*rho*S*V0^2*CLd]/(m*V0);
B2 = [-T0*cos(thetaT0)*xCG,  1/2*rho*S*V0^2*Dref*Cmd]/Jy;
Brb = [B1;B2];

Cx = eye(2);
CAccCG = [-1/m*(1/2*rho*S*V0^2*CLa + T0*cos(thetaT0 + alph0)),  0];
CAcc = [CAccCG(1) + 1/Jy*(xCG - (listAcc-1)'*l)*1/2*rho*S*V0^2*Dref*Cma,  zeros(length(listAcc),1)];
Crb = [Cx; CAccCG; CAcc];

Dx = zeros(2);
DAccCG = -1/m*[T0*cos(thetaT0 + alph0),  1/2*rho*S*V0^2*CLd];
DAcc = ones(length(listAcc),1)*DAccCG  +  1/Jy*(xCG - (listAcc-1)'*l)*[-T0*cos(thetaT0)*xCG,  1/2*rho*S*V0^2*Dref*Cmd];
Drb = [Dx; DAccCG; DAcc];

%% Naming
% states
xnames = {'\alpha','q'};
% inputs
unames = {'\theta_T';'\delta_f'};
% outputs
ynames = {'\alpha','q','a_zCG'};
for acc = listAcc
    ynames = [ynames, strcat('a_{z',num2str(acc),'}')];
end
names.x = xnames';
names.y = ynames';
names.u = unames';