p = n; % Order of precision (<= n)

Mm = diag(dm);
Md = diag(1/Vx*qSCLa);
Ms = zeros(n);
D4 = 3*eye(n);
D4(2:n,1:n-1) =D4(2:n,1:n-1) -4*eye(n-1);
D4(3:n,1:n-2) =D4(3:n,1:n-2)+ 1*eye(n-2);
D4 = 1/dl^4*(D4+D4');
D1 = zeros(n);
D1(2:n,1:n-1) = D1(2:n,1:n-1) + eye(n-1);
D1 = 1/(2*dl)*(D1+D1');
Ms = diag(E*I_struct)*D4-diag(qSCLa)*D1;

% 1 -4 6 -4 1
%%
%A = [zeros(n),eye(n); inv(Mm)*Ms, inv(Mm)*Md];
[modes,eigen] = eig(inv(Mm)*Ms);
%modes = real(modes);
eigen = diag(eigen);

figure
hold on
for i = n-4:n
    plot(modes(:,i));
end