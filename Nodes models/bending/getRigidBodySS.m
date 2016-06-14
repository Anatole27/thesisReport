function [Arb,Brb] = getRigidBodySS(qS,CL0,CLa,Cm0,Cma,xCM,CD0,kD,Dref,Vx,gamma,l,dm,x)

% Computes SS matrices for rigib body system at x
% INPUTS
% qS : dynamic pressure x reference surface (N)
% CL0, CLa : lift coefficients (zero value and slope) (alpha in rad !!)
% Cm0, Cma : pitching moment coefficients (zero value and slope)
% xCM : x value where Cm0 and Cma are calculated.x(tail)=0, x(nose)=L (m)
% CD0, kD : zero lift drag and induced drag coefficient
% Dref : length reference (booster diameter) (m)
% Vx : Missile speed (m/s)
% gamma : flight path (rad)
% l : beam element length (m)
% dm : vector of nodes mass (kg) [nx1]

%OUTPUTS
% (Arb,Brb). Matrices of SS system of X = [y,y_dot,theta,theta_dot]

% Lift, Moment and Drag coefficients
syms alpha T thetaT
CL = CL0 + alpha*CLa;
Cm = Cm0 + alpha*Cma;
CD = CD0 + kD*CL^2;

% Mass properties
m = sum(dm); % Mass
W = 9.81*m; % Weight
[ICG,xCG] = inertia(dm,l); % Inertia and position of CG
x = xCG; % Where the pitching moment equation is formulated

% Equations of Motion
sustentation = qS*CL-W*cos(gamma)+T*sin(alpha+thetaT); % Acceleration normal to flight path
propulsion = -qS*CD + T*cos(alpha+thetaT) - W*sin(gamma); % Acceleration along flight path
pitch = qS*Dref*Cm - T*sin(thetaT)*x + qS*CL*cos(alpha)*(xCM-x) + qS*CD*sin(alpha)*(xCM-x) - W*cos(alpha+gamma)*(xCG-x); % Rotation acceleration

% Solving
solution = solve(sustentation,propulsion,pitch);
alpha0 = double(solution.alpha);
T0 = double(solution.T);
thetaT0 = double(solution.thetaT);

% Linearizing around trim point
% x_dot = f(x,u)
% x = [y,y_dot,theta,theta_dot]
% u = thetaT
syms y y_dot theta theta_dot
fy = subs(1/m*sustentation,{T,alpha},{T0,theta-gamma+y_dot/Vx});
ftheta = subs(1/ICG*pitch,{T,alpha},{T0,theta-gamma+y_dot/Vx});
f = [y_dot; fy; theta_dot; ftheta];
df = jacobian(f,[y,y_dot,theta,theta_dot,thetaT]);
df = eval(subs(df,{y,y_dot,theta,theta_dot,thetaT},{0,0,alpha0+gamma,0,thetaT0}));

% Compute SS couple (A,B)
Arb = df(:,1:4);
Brb = df(:,5);