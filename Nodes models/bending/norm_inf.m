%function norm = norm_inf(om,z,bm,cmq,cmr,cma);
% Author : Wodek K. Gawronski
%This function finds approximate Hinf norm
%for each mode of a structure with displacement, rate, and acceleration
%sensors
%
% Input parameters :
%om - vector of natural frequencies
% z - vector of modal damping
% bm - modal matrix of actuator location
% cmq - modal matrix of displacement sensor location
% cmr - modal matrix of rate sensor location
% cma - modal matrix of accelerometer location
%
% Output parameter :
% norm - hinfo norm

function norm = norm_inf(om,z,bm,cmq,cmr,cma);

om2=diag(om.*om);
bb=diag(bm*bm');
cc=diag(cma'*cma*om2+cmr'*cmr+cmq'*cmq*inv(om2));
h = sqrt(bb.*cc)/2;
h=h./z;
norm=h./om;