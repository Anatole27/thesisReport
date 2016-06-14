function [Arb,Brb,Crb,Drb,alph0,T0,thetaT0,names] = getRigidBodySS(qS,CL0,CLa,Cm0,Cma,xCM,xIMU,CD0,kD,Dref,V0,X0,Z0,gamm0,l,dm)

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
syms Z X theta W U q T thetaT dalph_gust gamm V
alph = atan(W/U) + dalph_gust;
CL = CL0 + alph*CLa;
Cm = Cm0 + alph*Cma;
CD = CD0 + kD*CL^2;

%% State Equations
speed = V == sqrt(W^2+U^2);
slope = gamm == theta - alph;
Fzbody = qS*(-CL*cos(alph) - CD*sin(alph)) + m*g*cos(theta) - T*sin(thetaT);
Fxbody = qS*( CL*sin(alph) - CD*cos(alph)) - m*g*sin(theta) + T*cos(thetaT);
My = qS*Dref*Cm - T*sin(thetaT)*xCG + qS*CL*cos(alph)*(xCM-xCG) + qS*CD*sin(alph)*(xCM-xCG);
wdot = 1/m*Fzbody + U*q; % normal to body
udot = 1/m*Fxbody - W*q; % along body
qdot = 1/ICG*My; % Rotation acceleration /CG
system = [speed; slope; wdot; udot; qdot];
systemEq = subs(system,{gamm,V,dalph_gust,q},{gamm0,V0,0,0});

% Approximate solution (gamm0 = 0)
alph0  = (m*g/qS -CL0)/CLa;
U0 = V0*cos(alph0);
W0 = V0*sin(alph0);
theta0 = gamm0 + alph0;
T0 = qS*(CD0 + kD*(CL0 + CLa*alph0)^2);
thetaT0 = 1/(T0*xCG)*(qS*Dref*(Cm0+Cma*alph0) + qS*(CL0 + CLa*alph0)*(xCM-xCG));

% Solving
[T0,U0,W0,thetaT0,theta0] = vpasolve(systemEq,[ T, U, W, thetaT, theta],[T0,U0,W0,thetaT0,theta0]);
U0 = double(U0);
W0 = double(W0);
theta0 = double(theta0);
T0 = double(T0);
thetaT0 = double(thetaT0);
alph0 = atan(W0/U0);

% Linearizing around trim point
% x_dot = f(x,u_in)
% y = g(x,u_in)
% x = [z,x,theta,w,u,q]
% y = [x,
% u_in = thetaT
states = [Z;X;theta;W;U;q];
states0 = [Z0;X0;theta0;W0;U0;0];
inputs = [thetaT;dalph_gust];
inputs0 = [thetaT0;0];

% Keeping only [w,u,theta,wdot,udot,q], [thetaT, dalph_gust]
fpos = [W*cos(theta) - U*sin(theta);
    U*cos(theta) + W*sin(theta);
    q];
fspeed = subs(system(3:5),{gamm,V,T},{theta - alph,sqrt(W^2+U^2),T0});
gx = states;

% Extra outputs
az_IMU = subs(1/m*Fzbody - 1/ICG*My*(xCG-xIMU),{gamm,V,T},{theta - alph,sqrt(W^2+U^2),T0}); % Accel /z body at IMU pos
Fzsteer = -qS*CL + m*g*cos(gamm) - T*sin(thetaT+alph);
az = subs(1/m*Fzsteer,{gamm,V,T},{theta - alph,sqrt(W^2+U^2),T0});
gextra = [az;az_IMU];

% f(x,u) = x_dot
f = [fpos;fspeed];
df = jacobian(f,[states;inputs]); % Linearize

% g(x,u) = y
g = [gx;gextra];
dg = jacobian(g,[states;inputs]);

% Linearize at trim point
df = eval(subs(df,[states;inputs],[states0;inputs0]));
dg = eval(subs(dg,[states;inputs],[states0;inputs0]));

% Compute SS system
Arb = df(:,1:6);
Brb = df(:,7:8);
Crb = dg(:,1:6);
Drb = dg(:,7:8);

%% Naming
% states
xnames = {'z','x','\theta','w','u','q'};
% inputs
unames = {'\theta_T','\delta\alpha_{gust}'};
% outputs
ynames = {'z','x','\theta','w','u','q','a_z','a_zIMU'};
names.x = xnames';
names.y = ynames';
names.u = unames';