function plotSigmaModes(sigma)

[n,p] = size(sigma);
figure
for i = 1:n
    subplot(n,1,i);
    bar(sigma(i,:));
end

figure
sig_max = max(sigma);
bar(sig_max);