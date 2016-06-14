function [EI,l,n,dm,Diam,S,rho,CL0,CLa,CLd,Cm0,Cma,Cmd,xAC,xCG,xFins,Jy,CD0,kD,Lref,V0,gamma,acc] = loadModelEB(n)

%% Missile parameters initialisation
%
% Author: Anatole VERHAEGEN
%
%INPUTS
%
% * n: number of nodes 
%
% OUPUTS
%
% * EI: [n-1x1] vector of stiffness (N.m^2)
% * l: beam element length (m)
% * n: number of nodes 
% * dm: [nx1] vector of point masses (kg)
% * Diam: [nx1] vector of diameters (m)
% * S: reference surface (m2)
% * rho: air density (kg/m3)
% * CL0, CLa: lift coefficient zero value and slope (alpha in rad)
% * CLd: fins lift coefficient slope (rad-1)
% * Cm0, Cma: pitching moment coefficient zeros value and slope (alpha in
% rad)
% * Cmd: fins pitching moment coefficient (rad-1)
% * xAC: aerodynamic center position (m)
% * xCG: CG position (m)
% * xFins: fins position (m)
% * Jy: rotational inertia at CG about y-axis (kg.m2)
% * CD0, kD: drag coefficient zero lift value and induced drag coeff
% * Lref: reference length = booster diameter (m)
% * V0: speed (m/s)
% * gamma: flight path (rad)
% * acc: longax (m/s2)

%% Mass properties

% n point mass nodes and n-1 weightless beam:
%   *-------*-------*-------*-------*-------*-------*-------*
%   |       |<----->|
%  node       beam
%   dm         EI

dart_mass = 140;
dart_length = 2.7;
booster_mass = 310;
booster_length = 2.2;
mass = booster_mass+dart_mass;
total_length = dart_length + booster_length;
dm = zeros(n,1);
l = total_length/(n-1); % beam element length
dm_dart = dart_mass/dart_length*l; % mass of a node on the dart
dm_booster = booster_mass/booster_length*l; % mass of a node on the booster

% Distribute mass regarding the part of the rocket
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

[Jy,xCG] = inertia(dm,l); % Rotational inertia and CG position

%% Structural Properties

D_booster = 0.36;
d_booster = 0.35525;
D_dart = 0.18;
d_dart = 0.17525;

%E = 70*10^9; % Aluminium
E = 180*10^9; % Carbon fiber composite 70/30, unidirectional

% Second moments of area for pipes
I_booster = pi*(D_booster^4-d_booster^4)/64;
I_dart = pi*(D_dart^4-d_dart^4)/64;

I_struct = zeros(n-1,1);
Diam = zeros(n,1);
% Distribution of second moment of area regarding the part of the rocket
for i = 1:n-1
    if (i-1/2)*l < booster_length
        I_struct(i) = I_booster; % booster mass
    else
        I_struct(i) = I_dart; % dart mass
    end
end

% Distribution of diameter regarding the part of the rocket
for i = 1:n
    if (i-1)*l < booster_length
        Diam(i) = D_booster; % booster mass
    else
        Diam(i) = D_dart; % dart mass
    end
end

EI = E.*I_struct; % Vector of bending stiffnesses.

%% Aerodynamics and flight parameters

% Flight parameters
Hp = 0; %pressure alt, feet
rho = 1.21; % Sea level air density
M = 2; %Mach number
r = 287; % Specific gas constant for air
T = 15 + 273.15 - 2*Hp/1000; % ISA temperature in Kelvins
gamma_air = 1.4; % Heat capacity ratio
V0 = M*sqrt(gamma_air*r*T); % Missile speed
acc = 15*9.81; % Acceleration
gamma = 0*pi/180; % Flight path

% Body aerodynamics
Lref = D_booster; % Length surface
S = pi*Lref^4/4; % Reference surface
CL0 = 0; CLa = 22; % Lift coefficients
xAC = xCG - 0.8; % Aerodynamic center position
Cm0 = 0; Cma = CLa*(xAC-xCG)/Lref; %  pitching moment coefficients
CD0 = 0.95; kD = 1; % Drag coefficients

% Fins aerodynamics
ARf = 1; % Fins aspect ratio
Sfins = 2*0.15^2; % Fins surface
CLd = pi*ARf*Sfins/(1+sqrt(1+(pi*ARf/(2*0.7))^2))/S; % Fins lift coeff
xFins = 2.5; % Fins position
Cmd = CLd*(xFins-xCG)/Lref; % Fins pitching moment coeff
