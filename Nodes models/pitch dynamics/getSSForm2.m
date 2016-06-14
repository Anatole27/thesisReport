function [Am,Bm,Cm,Dm,Vm,names] = getSSForm2(om,z,Bm,Cmq,Cmr,Cma,Dq,Dr,Da,n,Phi)

Omega = diag(om);
Z = diag(z);
[n_modes,p] = size(Bm);

% X = [qm; qm_dot]
A = [zeros(n_modes), eye(n_modes); -Omega^2, -2*Z*Omega];
B = [zeros(n_modes,p); Bm];
C = [blkdiag(Cmq,Cmr); Cma];
D = [Dq;Dr;Da];
[r,~] = size(C);

% Xm = [qmi;qmoi]i with qmoi = zi*qmi + qmi_dot/wi. Vm*Xm = X.
Vm = zeros(2*n_modes,2*n_modes);
for i = 1:n_modes
    Vm(i,2*i-1) = 1;
    Vm(i+n_modes,2*i-1) = -om(i)*z(i);
    Vm(i+n_modes,2*i) = om(i);
end
Am = inv(Vm)*A*Vm;
Bm = inv(Vm)*B;
Cm = C*Vm;
Dm = D;

% Augmenting outputs
Cm = [Cm;[eye(n_modes) zeros(n_modes)];Phi*Vm(1:n_modes,:)]; %Sensor; wm; w
Dm = [Dm;zeros(n_modes+n,n)];

%% Naming
% states
for i = 1:n_modes
    xnames{2*i-1} = strcat('q_{m',num2str(i),'}');
    xnames{2*i} = strcat('q_{mo',num2str(i),'}');
end
% inputs
for i = 1:p
    unames{i} = strcat('F_{',num2str(i),'}');
end
% outputs
for i = 2:n-1
    ynames{i-1} = strcat('\epsilon_{',num2str(i),'}');
end
for i = 1:n
    ynames{i+n-2} = strcat('q_{',num2str(i),'}');
end
for i = 1:n
    ynames{i+2*n-2} = strcat('a_{z',num2str(i),'}');
end
for i = 1:n_modes
    ynames{i+3*n-2} = strcat('w_{m',num2str(i),'}');
end
for i = 1:n
    ynames{i+3*n+n_modes-2} = strcat('w_{',num2str(i),'}');
end
names.x = xnames';
names.y = ynames';
names.u = unames';