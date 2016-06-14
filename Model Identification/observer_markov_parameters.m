n = 2;
p = ceil(n/m); % p*m >= n and (r+m)*p+r <= l
p = 2;

% Input/Output
y = output'; % Output vector (m x l)
u = input'; % Input vector (r x l)
v = [u; y]; % Input-Output vector ((r + m) x l)

% V Matrix
V_complete = u; % Roman bold V matrix
for i = 1:l-1
    v_shift = [zeros(m+r,i) v(:,1:l-i)];
    V_complete = [V_complete; v_shift];
end
V = V_complete(1:(r+m)*p+r,:); % Truncated version of Roman bold V

% Eliminating initial conditions influence
y_bar = y(:,p+1:end);
V_bar = V(:,p+1:end);

% Markov parameters least square approximation without initial conditions
Y_bar = y_bar*pinv(V_bar);

% Extracting first & second part of Y_bar :
% 1
clear Y_bar1 Y_bar2
Y_bar1{1} = Y_bar(:,1:r);
Y_bar2{1} = zeros(m,m);
for k = 2:p+1
    Y_bar1{k} = Y_bar(:,r+1+(k-2)*(r+m):2*r+(k-2)*(r+m));
    Y_bar2{k} = -Y_bar(:,2*r+1+(k-2)*(r+m):2*r+m+(k-2)*(r+m));
end
for k = p+2:l+1
    Y_bar1{k} = zeros(m,r);
    Y_bar2{k} = zeros(m,m);
end

% Y_bar1 = cell2mat(Y_bar1);
% Y_bar2 = cell2mat(Y_bar2);

% figure
% hold on
% plot(Y_bar1);
% plot(Y_bar2);
% hold off