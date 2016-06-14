function [xCG,T0,Cma,Cmd] = addUncertaintyRB(xCG,T0,CLa,CLd,xFins,xAC,Dref)

%%  Create uncertain parameters for the rigid-body
%
% Author: Anatole VERHAEGEN
%
% This function will generate uncertainty on the center of gravity
% position, the thrust magnitude and therefore uncertainty on the pitching
% moment coefficients
%
% INPUTS:
%
% * xCG: center of gravity position (m)
% * T0: thrust magnitude (N)
% * CLa: lift coefficient slope (rad-1)
% * CLd: fins lift coefficient slope (rad-1)
% * xFins: fins position (m)
% * xAC: aerodynamic center position (m)
% * Dref: length reference (booster diameter) (m)
%
% OUTPUTS:
%
% * xCG: uncertain CG position (m)
% * T0: uncertain thrust magnitude (N)
% * Cma: uncertain pitching moment coeff (rad-1)
% * Cmd: uncertain fins pitching moment coeff (rad-1)

xCG = ureal('xCG',xCG,'Percentage',[-0.01, 10]);
T0 = ureal('T0',T0,'Percentage',[-10, 10]);
Cma = CLa*(xAC-xCG)/Dref;
Cmd = CLd*(xFins-xCG)/Dref;