function [Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names] = getRigidBodySS(S,rho,CL0,CLa,CLd,xFins,Cm0,Cma,xIMU,CD0,kD,Dref,V0,X0,Z0,gamm0,Vdot0,l,dm,listAcc)

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
[ICG,xCG] = inertia(dm,l); % Inertia and position of CG

% Lift, Moment and Drag coefficients
syms u gamm alph q thetaT deltaF T
CL = CL0 + alph*CLa + deltaF*CLd;
Cm = Cm0 + alph*Cma + deltaF*CLd*(xFins-xCG)/Dref;
CD = CD0 + kD*CL^2;
V = V0*(u+1);
qS = 1/2*rho*S*V^2;

%% State Equations
Vdot = -Vdot0 + 1/m*(-qS*CD + T*cos(alph+thetaT) - m*grav*sin(gamm));
gammdot = 1/(m*V)*(qS*CL + T*sin(alph+thetaT) - m*grav*cos(gamm));
alphdot = q - gammdot;
qdot = 1/ICG*(qS*Dref*Cm - T*sin(thetaT)*xCG);
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

% Linearizing around trim point
% x_dot = f(x,u_in)
% y = g(x,u_in)
% x = [z,x,theta,w,u,q]
% y = [x,
% u_in = thetaT
states = [alph;q];
states0 = [alph0;0];
inputs = [thetaT;deltaF];
inputs0 = [thetaT0;0];

% f(x,u) = x_dot
f = subs([alphdot,qdot]',{T,gamm,u},{T0,gamm0,0});
df = jacobian(f,[states;inputs]); % Linearize

% g(x,u) = y
gx = states;
Fzsteer = -qS*CL + m*grav*cos(gamm) - T*sin(thetaT+alph);
Mpitch = qS*Dref*Cm - T*sin(thetaT)*xCG;
az = subs(1/m*Fzsteer,{T,gamm,u},{T0,gamm0,0});
for i = 1:length(listAcc)
    azAcc(i) = 1/m*Fzsteer - grav*cos(gamm) + 1/ICG*(xCG - (listAcc(i)-1)*l)*Mpitch;
    azAcc(i) = subs(azAcc(i),{T,gamm,u},{T0,gamm0,0});
end
gextra = [az;azAcc'];
g = [gx;gextra];
dg = jacobian(g,[states;inputs]);

% Linearize at trim point
df = eval(subs(df,[states;inputs],[states0;inputs0]));
dg = eval(subs(dg,[states;inputs],[states0;inputs0]));

% Compute SS system
Arb = df(:,1:2);
Brb = df(:,end-1:end);
Crb = dg(:,1:2);
Drb = dg(:,end-1:end);

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