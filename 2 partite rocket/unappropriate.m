% Two partite rocket.
M = [m1 m2 0 0;
    -l1/2*m1 0 I1 0;
    -l2/2*m1 0 0 I2;
    1 -1 l1/2 l2/2];
D = zeros(4,2);
D(2:3,:) = lambda*[1 -1; -1 1];
K = [-q*CLa1*S -q*CLa2*S;
    l1/2*q*CLa1*S+k -k;
    l2/2*q*CLa1*S-k  k;
    0 0];
CpCq = [1 1; 0 -l1; l2/2 -l2/2; 0 0];

% Equations
red = [zeros(2) eye(2)];
A = [zeros(2) eye(2);
    -red*inv(M)*K -red*inv(M)*D];
A = A+1/100000*eye(4);
B = [zeros(2); red*inv(M)*CpCq];

% Augmenting system % Measure segment 2
C = [eye(4); zeros(1,4);[0 1 0 0;0 0 0 1]]; D = [zeros(4,2);0 1;zeros(2,2)];
G = ss(A,B,C,D);

% Actuator dynamics
s = zpk('s');
wc = 200;
w_act = 1/(s/wc+1);
Win(2,2) = w_act;
Win(1,1) = 1;

G = G*Win;

% Control
CL_inap = lft(G,K1);