function plotSigma(Omega,Z,Bm,Cmq,Cmv)
n = length(Omega);
[~,p] = size(Bm);
[r,~] = size(Cmq);
A = [zeros(n) eye(n); -Omega^2 -2*Z*Omega];
B = [zeros(n,p); Bm];
C = [Cmq Cmv];
D = zeros(r,p);

sys = ss(A,B,C,D);
sigma(sys)
% figure
% hold on
% for i = 1:p
%     for j = 1:r 
% sigma(sys(i,j));
%     end
% end
% hold off
%pzmap(sys)
% damp(sys)