function corrH = correlationHankelMatrix(Y,k,alpha,tau,xi,zeta)

% m*alpha must be greater than the system order
% beta should be chosen such that beta = l+1-alpha
% xi easiest : xi = 0;
% tau must be bigger than alpha to avoid overlap
% zeta

if tau < alpha
    error('tau must be greater than alpha');
end

m = length(Y{1}(:,1));
l = length(Y);
beta = l+1-(alpha+tau+zeta+xi+k);

% Init
corrH = zeros((xi+1)*alpha*m,(zeta+1)*alpha*m);

% Hankel matrix k =0

for i = 1:xi+1
    for j = 1:zeta+1
        beta = l-alpha-(k+(i+j-2)*tau)+1;
        H0 = hankelMatrix(Y,0,alpha,beta);
        Hk = hankelMatrix(Y,k+(i+j-2)*tau,alpha,beta); % Hankel matrix k
        Rhh = Hk*H0' % Correlation Rhh
        corrH((i-1)*alpha*m+1 : i*alpha*m , (j-1)*alpha*m+1 : j*alpha*m) = Rhh;
    end
end