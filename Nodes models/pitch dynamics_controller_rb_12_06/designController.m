function [K,sysCL] = designController(gSys,N_MEAS,N_CON,N_Z,N_W)

sys = gSys(end-(N_MEAS + N_Z)+1:end , end-(N_CON + N_W)+1:end); % Keep useful I/O only
[r,p] = size(sys);

%% Adding fictive damping for Hinf synthesis
n = length(sys.a);
sys.a = sys.a - 0.001*eye(n);

% %% TO DELETE
% warning('deleting x & z');
% [A,B,C,D] = ssdata(sys);
% A = A(3:end,3:end);
% B = B(3:end,:);
% C = C(:,3:end);
% sysyolo = ss(A,B,C,D);
% sysyolo.u = sys.u; sysyolo.y = sys.y; sysyolo.StateName = sys.StateName(3:end);
% sys = sysyolo;

%% Weighting functions :
s = zpk('s');
tauMax = 0.001;
addPole = 1/(1 + tauMax*s);

w_thetaT = zpk(1/( 0.1 )); % Out control

tau_az = 0.1;
%w_daz = 1/( 1/(1 + tau_az*s) )*addPole; % Out diff
w_daz = zpk(1);
Win = zpk(eye(p)); Win.u = sys.u; Win.y = sys.u;

Wout = zpk(eye(r)); Wout.u = sys.y; Wout.y = sys.y;
Wout('a_{zref}-a_z','a_{zref}-a_z') = w_daz;
Wout('\theta_T','\theta_T') = w_thetaT;

sysw = Wout*sys*Win;

%% Hinf synthesis
[K,~,GAM,INFO] = hinfsyn(sysw,N_MEAS,N_CON,'method','lmi');
K.u = sys.y(end-N_MEAS+1:end);
K.y = sys.u(end-N_CON+1:end);
%K = minreal(K);
sysCL = lft(gSys,K);