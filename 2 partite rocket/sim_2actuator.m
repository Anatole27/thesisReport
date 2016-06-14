% Time
slowmo = 5;
dt = 1/(24*slowmo);
tend = 2;
time = 0:dt:tend;

% Input
sim('windgust_sim',time);
Fin = windgust(:,2)*3000;

% Sim
y = lsim(CL,Fin,time);
y2 = lsim(CL2,Fin,time);
alpha1 = y(:,1);
alpha2 = y(:,2);
alpha1_2 = y2(:,1);
alpha2_2 = y2(:,2);
Fc = y(:,5);
Fc1 = y2(:,5);
Fc2 = y2(:,6);

% Nose and tail
nose = [-l2*sin(alpha2) l2*cos(alpha2)];
tail = [l1*sin(alpha1) -l1*cos(alpha1)];
nose2 = [-l2*sin(alpha2_2) l2*cos(alpha2_2)];
tail2 = [l1*sin(alpha1_2) -l1*cos(alpha1_2)];

fig = figure;
set(fig,'Position',[0 0 300 800])
subplot(2,1,1);
hold on
missile = plot(zeros(3,1),zeros(3,1),'LineWidth',2);
perturbation = quiver(0,0,0,0,0,'MaxHeadSize',10);
control = quiver(0,0,0,0,0,'MaxHeadSize',10);
energy = plot([0;0],[0;0],'r','LineWidth',4);
width = (l1+l2)*1.5;
axis equal
xlim([-width/2 width/2])
ylim([-width/2 width/2])

subplot(2,1,2);
hold on
missile2 = plot(zeros(3,1),zeros(3,1),'LineWidth',2);
perturbation2 = quiver(0,0,0,0,'MaxHeadSize',10);
control2 = quiver([0; 0],[0; 0],[0; 0],[0; 0],0,'MaxHeadSize',10);
energy2 = plot([0;0],[0;0],'r','LineWidth',4);
axis equal
xlim([-width/2 width/2])
ylim([-width/2 width/2])

scale = width*0.45/max([abs(Fc);abs(Fc1);abs(Fc2)]);

nrg = 0;
nrg2 = 0;
scale_nrg = width*0.8/max([sum(1/2*Fc.^2);sum(1/2*Fc1.^2+1/2*Fc2.^2)]);

aviobj = VideoWriter('sim_gimbal.avi');
aviobj.FrameRate = 24;
open(aviobj);

for k = 1:length(time)
    set(missile,'XData',[nose(k,1); 0; tail(k,1)],'YData',[nose(k,2); 0; tail(k,2)]);
    set(perturbation,'XData',nose(k,1),'YData',nose(k,2),'UData',-Fin(k)*scale);
    set(control,'XData',tail(k,1),'YData',tail(k,2),'UData',-Fc(k)*scale);
    set(energy,'XData',[-width*0.4 -width*0.4+nrg*scale_nrg],'YData',[-width*0.45 -width*0.45]);
    drawnow
    
    set(missile2,'XData',[nose2(k,1); 0; tail2(k,1)],'YData',[nose2(k,2); 0; tail2(k,2)]);
    set(perturbation2,'XData',nose2(k,1),'YData',nose2(k,2),'UData',-Fin(k)*scale);
    set(control2,'XData',[tail2(k,1);0],'YData',[tail2(k,2);0],'UData',[-Fc1(k)*scale;-Fc2(k)*scale]);
    set(energy2,'XData',[-width*0.4 -width*0.4+nrg2*scale_nrg],'YData',[-width*0.45 -width*0.45]);
    drawnow
    
    nrg = nrg+1/2*Fc(k)^2;
    nrg2 = nrg2+1/2*Fc1(k)^2+1/2*Fc2(k)^2;
    
    % Record
    F = getframe(fig);
    writeVideo(aviobj,F);
end
close(aviobj);