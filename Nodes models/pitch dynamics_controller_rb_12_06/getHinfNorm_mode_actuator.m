function Gij = getHinfNorm_mode_actuator(B,C,i,j,om,z)

Bij = B(2*i-1:2*i,j);
Ci = C(:,2*i-1:2*i);

Gij = norm(Bij)*norm(Ci)/(2*z(i)*om(i));