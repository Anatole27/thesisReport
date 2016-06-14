function [Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names] = getRigidBodySS(S,rho,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,V0,X0,Z0,gamm0,l,dm)

% Computes SS matrices for rigib body system at x
% INPUTS
% qS : dynamic pressure x reference surface (N)
% CL0, CLa : lift coefficients (zero value and slope) (alph in rad !!)
% Cm0, Cma : pitching moment coefficients (zero value and slope)
% xCM : x value where Cm0 and Cma are calculated.x(tail)=0, x(nose)=L (m)
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
g = 9.81;
[ICG,xCG] = inertia(dm,l); % Inertia and position of CG

% Lift, Moment and Drag coefficients
syms u gamm alph q thetaT dalph_gust T
CL = CL0 + (alph+dalph_gust)*CLa;
Cm = Cm0 + (alph+dalph_gust)*Cma;
CD = CD0 + kD*CL^2;
V = V0*(u+1);
qS = 1/2*rho*S*V^2;

%% State Equations
udot = 1/(m*V0)*(-qS*CD + T*cos(alph+thetaT) - m*g*sin(gamm));
gammdot = 1/(m*V)*(qS*CL + T*sin(alph+thetaT) - m*g*cos(gamm));
alphdot = q - gammdot;
qdot = 1/ICG*(qS*Dref*Cm - T*sin(thetaT)*xCG + qS*CL*cos(alph)*(xCM-xCG) + qS*CD*sin(alph)*(xCM-xCG));
system = [udot,gammdot,alphdot,qdot]';
systemEq = subs(system([1,2,4]),{gamm,u,dalph_gust,q},{gamm0,0,0,0});

% Approximate solution (gamm0 = 0)
alph0  = (m*g/(1/2*rho*S*V0^2) -CL0)/CLa;
T0 = (1/2*rho*S*V0^2)*(CD0 + kD*(CL0 + CLa*alph0)^2);
thetaT0 = 1/(T0*xCG)*(1/2*rho*S*V0^2)*(Dref*(Cm0+Cma*alph0) + (CL0 + CLa*alph0)*(xCM-xCG));

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
states = [u;gamm;alph;q];
states0 = [0;gamm0;alph0;0];
inputs = [thetaT;dalph_gust];
inputs0 = [thetaT0;0];

% f(x,u) = x_dot
f = subs([udot,gammdot,alphdot,qdot]',{T},{T0});
df = jacobian(f,[states;inputs]); % Linearize

% g(x,u) = y
gx = states;
Fzsteer = -qS*CL + m*g*cos(gamm) - T*sin(thetaT+alph+dalph_gust);
az = subs(1/m*Fzsteer,{T},{T0});
gextra = az;
g = [gx;gextra];
dg = jacobian(g,[states;inputs]);

% Linearize at trim point
df = eval(subs(df,[states;inputs],[states0;inputs0]));
dg = eval(subs(dg,[states;inputs],[states0;inputs0]));

% Compute SS system
Arb = df(:,1:4);
Brb = df(:,end-1:end);
Crb = dg(:,1:4);
Drb = dg(:,end-1:end);

%% Naming
% states
xnames = {'u','\gamma','\alpha','q'};
% inputs
unames = {'\theta_T','\delta\alpha_{gust}'};
% outputs
ynames = {'u',';\gamma','\alpha','q','a_z'};
names.x = xnames';
names.y = ynames';
names.u = unames';