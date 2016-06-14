%% Bodes
%% azref/azerr
figure
hold on
bode(CLraw.NominalValue('a_{zerr}','a_{zref}'));
bode(CLnotch.NominalValue('a_{zerr}','a_{zref}'));
bode(CLstrain.NominalValue('a_{zerr}','a_{zref}'));
bode(CLgyr.NominalValue('a_{zerr}','a_{zref}'));
bode(CLacc.NominalValue('a_{zerr}','a_{zref}'));
legend('Raw','Notch','Strain','Gyro','Acc')

%% Vibration creation by rocket motor
figure
hold on
bode(CLraw.NominalValue('a_{z83v}','\theta_{Tpert}'));
bode(CLnotch.NominalValue('a_{z83v}','\theta_{Tpert}'));
bode(CLstrain.NominalValue('a_{z83v}','\theta_{Tpert}'));
bode(CLgyr.NominalValue('a_{z83v}','\theta_{Tpert}'));
bode(CLacc.NominalValue('a_{z83v}','\theta_{Tpert}'));
legend('Raw','Notch','Strain','Gyro','Acc')

%% theta demand for azref
figure
hold on
bode(CLraw.NominalValue('\theta_T','a_{zref}'));
bode(CLnotch.NominalValue('\theta_T','a_{zref}'));
bode(CLstrain.NominalValue('\theta_T','a_{zref}'));
bode(CLgyr.NominalValue('\theta_T','a_{zref}'));
bode(CLacc.NominalValue('\theta_T','a_{zref}'));
legend('Raw','Notch','Strain','Gyro','Acc')

%% rocket engine noise to actuator position (parasites)
figure
hold on
bode(CLraw.NominalValue('\theta_T','\theta_{Tpert}'));
bode(CLnotch.NominalValue('\theta_T','\theta_{Tpert}'));
bode(CLstrain.NominalValue('\theta_T','\theta_{Tpert}'));
bode(CLgyr.NominalValue('\theta_T','\theta_{Tpert}'));
bode(CLacc.NominalValue('\theta_T','\theta_{Tpert}'));
legend('Raw','Notch','Strain','Gyro','Acc')