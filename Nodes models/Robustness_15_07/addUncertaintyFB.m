function [om,z] = addUncertaintyFB(om,z)

for i = 1:length(om)
    uom(i) = ureal(strcat('w',num2str(i)),om(i),'Percentage',[-10,10]);
    uz(i) = ureal(strcat('z',num2str(i)),z(i),'Percentage',[-20,20]);
end

    om = uom;
    z = uz;