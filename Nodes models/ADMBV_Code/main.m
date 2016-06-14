addpath('./Controllers_Generation','./Flexible_Missile_Model_Generation')

% Flexible missile model generation
display('Model generation')
initMissileFullIO

% Controllers generation
display('Controllers design')
initControllers

display('Done')
