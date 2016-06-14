%% This script will use the missile characteristics to compute a state space realization of flexible body
% Author : Anatole VERHAEGEN
N = 300;
om_rec = zeros(5,N);
N = 7:300;
for n = 7:300
    n
    [EI,l,n,dm,Diam,S,rho,CL0,CLa,Cm0,Cma,xCM,xCG,xIMU,CD0,kD,Dref,Vx,gamma,X0,Z0] = loadModel_EB(n);
    
    % Get 2nd order structural model
    [M,D,K,Bo,Coq,Cor,Coa,Dq,Dr,Da] = getMDK_Leckie(EI,l,n,dm,Diam);
    
    % Put into modal representation without rigid-body modes (X = [qm,qm_dot])
    % High order modes are truncated
    n_modes = 5; % number of structural modes kept
    [Mm,Dm,Km,Bm,Cmq,Cmr,Cmacc,om,Phi,z] = getModalRepTrunc(M,D,K,Bo,Coq,Cor,Coa,n_modes);
    om_rec(:,n) = om;
end
%%
figure; hold on;
scatter(N',om_rec(i,N)','xk');
grid minor
xlabel('Node or beam number $i$','Interpreter','latex')
ylabel(ax(1),'Mass of nodes $m_i$ ($kg$)','Interpreter','latex')
ylabel(ax(2),'Bending stiffness of beams $E_iI_i$ ($N.m^2$)','Interpreter','latex')
legend('Mass of nodes','Bending stiffness of beams')
print('~/Documents/Etudes/Cranfield University/Thesis/Reports/figures/beamStructDimGraph', '-dpng', '-r300'); %<-Save as PNG with 300 DPI