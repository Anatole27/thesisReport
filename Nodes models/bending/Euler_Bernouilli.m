p = n; % Order of precision (<= n)

Mm = diag(dm);
Md = diag(1/Vx*qSCLa);
Ms = zeros(n);
D = zeros(n,p);
for k = 1:n
    for i = 1:n
        for j = 1:p
            D(i,j) = ((i-k)*dl)^(j-1)/factorial(j-1);
        end
    end
    Dinv = pinv(D);
    Ms(k,:) = E*I_struct(k)*Dinv(5,:)-qSCLa(k)*Dinv(2,:);
end
%%
%A = [zeros(n),eye(n); inv(Mm)*Ms, inv(Mm)*Md];
[modes,eigen] = eig(inv(Mm)*Ms);
modes = real(modes);
eigen = diag(eigen);

for i = 1:length(eigen)
    if real(eigen(i)) < 0 && abs(imag(eigen(i))) < abs(real(eigen(i)))
        plot(real(modes(1:n,i)))
        pause(0.5)
        sqrt(-real(eigen(i)))/(2*pi)
    end
end