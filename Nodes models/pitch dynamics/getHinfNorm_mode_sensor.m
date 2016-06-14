function Gij = getHinfNorm_mode_sensor(B,C,i,j,om,z)

Bi = B(2*i-1:2*i,:);
Cji = C(j,2*i-1:2*i);

Gij = norm(Bi)*norm(Cji)/(2*z(i)*om(i));