function [om,z] = addUncertaintyFB(om,z)

%%  Create uncertain parameters for the flexible-body
%
% Author: Anatole VERHAEGEN
%
% This function will generate uncertainty on bending modes natural
% frequencies and damping ratios
%
% INPUTS:
%
% * om: vector of natural frequencies
% * z: vector of damping ratios
%
% OUTPUTS:
%
% * om: uncertain vector of natural frequencies
% * z: uncertain vector of damping ratios

%% Uncertainty : +-10% for nat freq, +-20% for damping ratios
for i = 1:length(om)
    uom(i) = ureal(strcat('w',num2str(i)),om(i),'Percentage',[-10,10]);
    uz(i) = ureal(strcat('z',num2str(i)),z(i),'Percentage',[-20,20]);
end

    om = uom;
    z = uz;