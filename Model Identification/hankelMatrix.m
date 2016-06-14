function H = hankelMatrix(Y,k,alpha,beta)

m = length(Y{1}(:,1));

H = zeros(m*alpha,m*beta);
for i = 1:alpha
    for j = 1:beta
        H((i-1)*m+1 : i*m , (j-1)*m+1 : j*m) = Y{k+i+j-1};
    end
end