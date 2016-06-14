function [EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xCM,xCG,Jy,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(n)
%% Missile structure and aerodynamics
% Author : Anatole VERHAEGEN

% n point mass nodes and n-1 weightless beam :
%   *-------*-------*-------*-------*-------*-------*-------*
%   |       |<----->|
%  node       beam
%   dm         EI

%INPUTS
% n : number of nodes 
%
%OUPUTS
% EI : [n-1x1] vector of stiffness (N.m^2)
% l : beam element length (m)
% dm : [nx1] vector of point masses (kg)
% Diam : [nx1] vector of diameters (m)
% qS : dynamic pressure x reference surface (Booster cross-section) (N)
% CL0, CLa : lift coefficient zero value and slope (alpha in rad)
% Cm0, Cma : pitching moment coefficient zeros value and slope
% xCM : x position where Cm is calculated (m)
% CD0, kD : drag coefficient zero lift value and induced drag coeff
% Dref : Reference length = booster diameter (m)
% Vx : speed (m/s)
% gamma : flight path (rad)

%% Mass
dart_mass = 140;
dart_length = 2.7;
booster_mass = 310;
booster_length = 2.2;
mass = booster_mass+dart_mass;
total_length = dart_length + booster_length;
dm = zeros(n,1);
l = total_length/(n-1);
dm_dart = dart_mass/dart_length*l;
dm_booster = booster_mass/booster_length*l;

% Apply dm regarding the part of the rocket
for i = 2:n-1
    if (i-1)*l < booster_length
        dm(i) = dm_booster; % booster mass
    else
        dm(i) = dm_dart; % dart mass
    end
end
dm(1) = dm_booster/2;
dm(n) = dm_dart/2;

dm = dm/sum(dm)*mass; % Adjust mass to total mass

%% Stiffness

D_booster = 0.36;
d_booster = 0.35525;
D_dart = 0.18;
d_dart = 0.17525;
%E = 70*10^9; % Aluminium
E = 180*10^9; % Carbon fiber composite 70/30, unidirectional

I_booster = pi*(D_booster^4-d_booster^4)/64;
I_dart = pi*(D_dart^4-d_dart^4)/64;

I_struct = zeros(n-1,1);
Diam = zeros(n,1);
% Apply I_struct regarding the part of the rocket
for i = 1:n-1
    if (i-1/2)*l < booster_length
        I_struct(i) = I_booster; % booster mass
    else
        I_struct(i) = I_dart; % dart mass
    end
end

% Apply D regarding the part of the rocket
for i = 1:n
    if (i-1)*l < booster_length
        Diam(i) = D_booster; % booster mass
    else
        Diam(i) = D_dart; % dart mass
    end
end

EI = E.*I_struct;

%% Aerodynamics

Hp = 0; %pressure alt, feet
rho = 1.21; % Altitude 0
M = 2; %Mach
r = 287;
T = 15 + 273.15 - 2*Hp/1000; % ISA
gamma_air = 1.4;
Vx = M*sqrt(gamma_air*r*T);
q = 1/2*rho*Vx^2;
% Aerodynamics
S = pi*D_booster^4/4;
CL0 = 0; CLa = 22;
Cm0 = 0; 
%Cma = 44; % Data from paper (not appropriate for xCG too fore)
Cma = 75; % ThetaT0 = -alph0 (Mach 2)
% Cma = 81 % ThetaT0 = 0
CD0 = 0.95; kD = 1;
xCM = (1-0.84)*total_length;

% Geometry
Dref = D_booster;
gamma = 0*pi/180; % Flight path

X0 = 0;
Z0 = 0;

%% IMU position
xIMU = 3.8;

[Jy,xCG] = inertia(dm,l);