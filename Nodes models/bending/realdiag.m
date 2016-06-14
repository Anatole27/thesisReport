%function [Am,Bm,Cm,Vm] = realdiag(A,B,C);
% Author : Wodek K Gawronski
% This function finds the modal state-space representation of form 2
% The diagonal blocks of Am are in increasing order of imaginary part of
% the eigenvalues of A
% Vm is the transformation matrix from (A,B,C) to (Am,Bm,Cm)
%
function [Am,Bm,Cm,Vm] = realdiag(A,B,C)
[V,D] = eig(A);
[x,in] = sort(diag(abs(D)));
D = D(in,in);
V = V(in,in);
B = B(in,:);
C = C(:,in);
[Vm,Am] = cdf2rdf(V,D);
Bm = inv(Vm)*B;
Cm = C*Vm;