% Stabilize and make it fast
Kq = -0.65;
loop1 = feedback(rbsys,Kq,1,2);
loop1(:,'\theta_T') = 1/133*loop1(:,'\theta_T');
Ka = -12.2;
loop2 = feedback(loop1,Ka,1,6);
% Still need a feedforward static gain

% Hinf style (Spilious)
loop1H = loop1({'a_{zref}-a_z','a_{zref}','a_z','q'},:)
[K,CL,GAM,INFO] = hinfsyn(loop1H,3,1);
rbCL = lft(loop1H,K);