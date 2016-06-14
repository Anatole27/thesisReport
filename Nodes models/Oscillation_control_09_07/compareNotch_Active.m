% Comparison
%% Tracking
sigma(CLVibeSupp(10,1),'b',CLVibeNotch(10,1),'g')
legend('Active damping','Filtered input')
title('Singular Values of error = a_{zref}-a_z')
grid on
%% Step
figure
step(CLVibeSupp(10,1),CLVibeNotch(10,1))
legend('Active damping','Filtered input')
title('Step response of tracking error = a_{zref}-a_z')
grid on

%% Actuator demand
figure
sigma(CLVibeSupp('thetaT',1),CLVibeNotch('thetaT',1))
legend('Active damping','Filtered input')
title('Singular Values of \theta_T')
grid on

%% 