function [Am,Bm,Cm,Dm,Vm] = getSSForm2(om,z,Bm,Cmq,Cmr)

Omega = diag(om);
Z = diag(z);
[n,p] = size(Bm);

% X = [qm; qm_dot]
A = [zeros(n), eye(n); -Omega^2, -2*Z*Omega];
B = [zeros(n,p); Bm];
C = [Cmq Cmr];
[r,~] = size(C);
Dm = zeros(r,p);

% Xm = [qmi;qmoi]i with qmoi = zi*qmi + qmi_dot/wi. Vm*Xm = X.
Vm = zeros(2*n,2*n);
for i = 1:n
    Vm(i,2*i-1) = 1;
    Vm(i+n,2*i-1) = -om(i)*z(i);
    Vm(i+n,2*i) = om(i);
end
Am = inv(Vm)*A*Vm;
Bm = inv(Vm)*B;
Cm = C*Vm;