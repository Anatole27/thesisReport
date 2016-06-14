% Time
slowmo = 5;
dt = 1/(24*slowmo);
tend = 1;
time = 0:dt:tend;

% Input
sim('windgust_sim',time);
Fin = windgust(:,2)*10000;

% Sim
y = lsim(CL,Fin,time);
y_inap = lsim(CL_inap,Fin,time);
alpha1 = y(:,1);
alpha2 = y(:,2);
alpha1_ex = 1/2*(alpha1+alpha2)+1/2*(alpha1-alpha2)*10;
alpha2_ex = 1/2*(alpha1+alpha2)+1/2*(alpha2-alpha1)*10;
alpha1 = alpha1_ex; %exagerating the bending
alpha2 = alpha2_ex;

alpha1_inap = y_inap(:,1);
alpha2_inap = y_inap(:,2);
alpha1_ex = 1/2*(alpha1_inap+alpha2_inap)+1/2*(alpha1_inap-alpha2_inap)*10;
alpha2_ex = 1/2*(alpha1_inap+alpha2_inap)+1/2*(alpha2_inap-alpha1_inap)*10;
alpha1_inap = alpha1_ex; %exagerating the bending
alpha2_inap = alpha2_ex;
Fc = y(:,5);
Fc_inap = y_inap(:,5);

% Nose and tail
nose = [-l2*sin(alpha2) l2*cos(alpha2)];
tail = [l1*sin(alpha1) -l1*cos(alpha1)];
nose_inap = [-l2*sin(alpha2_inap) l2*cos(alpha2_inap)];
tail_inap = [l1*sin(alpha1_inap) -l1*cos(alpha1_inap)];

fig = figure;
set(fig,'Position',[0 0 300 800])
subplot(2,1,2)
hold on
missile = plot(zeros(3,1),zeros(3,1),'LineWidth',2);
perturbation = quiver(0,0,0,0,0,'MaxHeadSize',10);
control = quiver(0,0,0,0,0,'MaxHeadSize',10);
energy = plot([0;0],[0;0],'r','LineWidth',4);
width = (l1+l2)*1.5;
axis equal
xlim([-width/2 width/2])
ylim([-width/2 width/2])
title('Two sensors')

subplot(2,1,1)
hold on
missile_inap = plot(zeros(3,1),zeros(3,1),'LineWidth',2);
perturbation_inap = quiver(0,0,0,0,0,'MaxHeadSize',10);
control_inap = quiver(0,0,0,0,0,'MaxHeadSize',10);
energy_inap = plot([0;0],[0;0],'r','LineWidth',4);
axis equal
xlim([-width/2 width/2])
ylim([-width/2 width/2])
title('Single sensor')

scale = width*0.45/max([abs(Fc)]);

nrg_ap = 0;
nrg_inap = 0;
scale_nrg = width*0.8/max(sum(1/2*Fc.^2),sum(1/2*Fc_inap.^2));

aviobj = VideoWriter('sim.avi');
aviobj.FrameRate = 24;
open(aviobj);

for k = 1:length(time)
    set(missile,'XData',[nose(k,1); 0; tail(k,1)],'YData',[nose(k,2); 0; tail(k,2)]);
    set(perturbation,'XData',nose(k,1),'YData',nose(k,2),'UData',-Fin(k)*scale);
    set(control,'XData',tail(k,1),'YData',tail(k,2),'UData',-Fc(k)*scale);
    %set(energy,'XData',[-width*0.4 -width*0.4+nrg_ap*scale_nrg],'YData',[-width*0.45 -width*0.45]);
    %drawnowclose all
    
    
    set(missile_inap,'XData',[nose_inap(k,1); 0; tail_inap(k,1)],'YData',[nose_inap(k,2); 0; tail_inap(k,2)]);
    set(perturbation_inap,'XData',nose_inap(k,1),'YData',nose_inap(k,2),'UData',-Fin(k)*scale);
    set(control_inap,'XData',tail_inap(k,1),'YData',tail_inap(k,2),'UData',-Fc_inap(k)*scale);
    %set(energy_inap,'XData',[-width*0.4 -width*0.4+nrg_inap*scale_nrg],'YData',[width*0.45 width*0.45]);
    %drawnow
    
    nrg_ap = nrg_ap+1/2*Fc(k)^2;
    nrg_inap = nrg_inap+1/2*Fc_inap(k)^2;
    
    % Record
    F = getframe(fig);
    writeVideo(aviobj,F);
end
close(aviobj);