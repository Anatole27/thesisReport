n = 100;
p = 10;
l = 3;
dl = l/n;
k = 50;
A = zeros(n,p);
for i = 1:n
    for j = 1:p
        A(i,j) = ((i-k)*dl)^(j-1)/factorial(j-1);
    end
end
X = rand(p,1);
for i = 1:n
    Y(i,1) = cos(i*dl*2*pi/l);
end
for i = 1:p
    X(i,1) = (2*pi/l)^(i-1)*cos(k*dl*2*pi/l + (i-1)*pi/2);
end
Xn = pinv(A)*Y;
Adag = pinv(A);
figure
hold on
plot(Y)
plot(A*X)