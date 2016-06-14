%% Beam dimensions
node = 1:n;
[ax,p1,p2] = plotyy(node,dm,node(1:n-1),EI);

p1.LineStyle = 'none'
p2.LineStyle = 'none'
p1.Marker = 'x'
p2.Marker = 'o'
grid minor
xlabel('Node or beam number $i$','Interpreter','latex')
ylabel(ax(1),'Mass of nodes $m_i$ ($kg$)','Interpreter','latex')
ylabel(ax(2),'Bending stiffness of beams $E_iI_i$ ($N.m^2$)','Interpreter','latex')
legend('Mass of nodes','Bending stiffness of beams')
xlim(ax(1),[0,21])
xlim(ax(2),[0,21])
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/beamStructDimGraph', '-dpng', '-r300'); %<-Save as PNG with 300 DPI