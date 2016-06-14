function [xCG,T0,Cma,Cmd] = addUncertaintyRB(xCG,T0,CLa,CLd,xFins,xAE,Dref)

 xCG = ureal('xCG',xCG,'Percentage',[-0.01, 10]);
T0 = ureal('T0',T0,'Percentage',[-10, 10]);
 Cma = CLa*(xAE-xCG)/Dref;
 Cmd = CLd*(xFins-xCG)/Dref;