loadModalSS
[missile,N_MEAS,N_CON,N_Z,N_W] = generalizeSystem(missile,3);
[K,missileCL] = designController(missile,N_MEAS,N_CON,N_Z,N_W);



% output = {'z','x','\theta','w','u'}';
% for i = 1:n
%     output = [output; strcat('w_',num2str(i))];
% end
% t = 0:0.001:1;
% nt = length(t);
% input = 0.1*cos(2*pi*80*t)';
% [y,t] = lsim(missile(output,1),input,t);
% z = y(:,1);
% x = y(:,2);
% theta = y(:,3);
% w = y(:,4);
% u = y(:,5);
% w_nodes = y(:,6:end);
% 
%displaySim(Z0,X0,gamma,alph0,Vx,xCG,thetaT0,input,T0,z,x,theta,w,u,w_nodes,t,1,n);