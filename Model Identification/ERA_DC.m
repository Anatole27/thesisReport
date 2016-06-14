% correlation Hankel Matrix H(0)
alpha = 3;
%beta = l+1-alpha;
beta = 3;
xi = 0;
zeta = 0;
tau = alpha;
H0 = correlationHankelMatrix(Y,0,alpha,tau,xi,zeta);

% SVD of H(0)
[R,Sigma,S] = svd(H0);

% System order :
i = 1;
hankelSV = diag(Sigma);
while(hankelSV(i+1) > hankelSV(i)*5/100)
    i = i+1;
end
order = i;

% Order reduction :
R = R(:,1:order);
S = S(:,1:order);
Sigma = Sigma(1:2,1:2);

% Minimum order realization of [A, Qc, Pa]
H1 = correlationHankelMatrix(Y,1,alpha,tau,xi,zeta);
A_hat = Sigma^(-1/2)*R'*H1*S*Sigma^(-1/2);
Qc = Sigma^(1/2)*S';
Pa = R*Sigma^(1/2);

% Controllability matrix
Qb = pinv(Pa)*H0;

% Minimum realization [A_hat B_hat C_hat]
B_hat = Qb(:,1:r);
C_hat = Pa(1:m,:);

% Modal realization
[V,Am] = eig(A_hat);
Bm = V^-1*B;
Cm = C*V;

% Modal Amplitude Coherence (MAC)
Q_bar = zeros(length(Bm(:,1)),length(Bm(1,:))*(l-alpha));
lambda = eye(length(Bm(:,1)));
for k = 0:l-alpha-1
    Q_bar(:,k+1:k+length(Bm(1,:))) = lambda*Bm;
    lambda = Am*lambda;
end
% No idea how to compute Q_hat

