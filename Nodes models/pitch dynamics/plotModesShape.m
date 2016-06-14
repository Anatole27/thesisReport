function plotModesShape(Phi,Omega2,n_modes)

figure
hold on
labels = {};
for i = 1:n_modes
    plot(Phi(:,i))
    labels{i} = strcat(num2str(sqrt(abs(real(Omega2(i,i))))),' rad/s');
end
legend(labels)

hold off