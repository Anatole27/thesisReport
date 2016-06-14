%% TRD/TMD & Ferm. Geo equations
% Init
A = zeros(n,n+1);
Bint = zeros(n,n-1);
Bext = zeros(n,n+1);
D = zeros(n);
C = zeros(n,n+1);
E = zeros(n);
Gint = zeros(n,n-1);
Gext = zeros(n,n+1);
H = zeros(n);
I = zeros(n);
J = zeros(n,n+1);
K = zeros(n,n);

% Definition of matrices
A(1:n,1:n) = 1/2*diag(dm);
A(1:n,2:n+1) = 1/2*diag(dm)+A(1:n,2:n+1);
Bint(1:n-1,1:n-1) = eye(n-1);
Bint(2:n,1:n-1) = Bint(2:n,1:n-1) - eye(n-1);
Bext(1:n,1:n) = eye(n);
Bext(1:n,2:n+1) = eye(n)+Bext(1:n,2:n+1);
D = diag(qSCLa);
C(1:n,1:n) = -diag(qSCLa/Vx)/2;
C(1:n,2:n+1) = -diag(qSCLa/Vx)/2 + C(1:n,2:n+1);
E = diag(dI);
Gint(1:n-1,1:n-1) = dl/2*eye(n-1);
Gint(2:n,1:n-1) = dl/2*eye(n-1) + Gint(2:n,1:n-1);
Gext(1:n,1:n) = -dl/2*eye(n);
Gext(1:n,2:n+1) = Gext(1:n,2:n+1) + dl/2*eye(n);
H(1:n,1:n) = -diag([k;0])-diag([0;k]);
H(1:n-1,2:n) = H(1:n-1,2:n) + diag(k);
H(2:n,1:n-1) = H(2:n,1:n-1) + diag(k);
I = 1/100*H;
J(1:n,1:n) = -eye(n);
J(1:n,2:n+1) = J(1:n,2:n+1) + eye(n);
K = dl*eye(n);

% Get Mass, Damping and Sitffness Matrices : (Mm, Md, Ms), Get force input
% matrix Mf :
Add = [A; E*K^-1*J]; 
Ad = [-C;-I*K^-1*J]; 
A = [-D*K^-2*J;-H*K^-1*J]; 
Afint = [-Bint; -Gint];
Afext = [Bext; Gext];
Adag = inv(Afint'*Afint)*Afint';

Mm = Add-Afint*Adag*Add;
Md = Ad-Afint*Adag*Ad;
Ms = A-Afint*Adag*A;
Mf = Afext-Afint*Adag*Afext; % (This system is oversampled)

% Get Modal SS matrices A,B,C,D,Cp,Cq
Mm_dag = inv(Mm'*Mm)*Mm';
[Phi,Omega] = eig(Mm_dag*Ms);
Am = [zeros(n+1),eye(n+1); Phi'*Mm_dag*Ms*Phi, Phi'*Mm_dag*Md*Phi];

figure
hold on
for i = n-3:n+1
    plot(real(Phi(:,i)))
end