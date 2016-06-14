% Time
slowmo = 10;
dt = 1/(24*slowmo);
tend = 0.5;
time = 0:dt:tend;

% Input
sim('windgust_sim',time);
Fin = windgust(:,2)*3000;

% Sim
y = lsim(CL_inap,Fin,time);
alpha1 = y(:,1);
alpha2 = y(:,2);
alpha1_ex = 1/2*(alpha1+alpha2)+1/2*(alpha1-alpha2)*100;
alpha2_ex = 1/2*(alpha1+alpha2)+1/2*(alpha2-alpha1)*100;
alpha1 = alpha1_ex; %exagerating the bending
alpha2 = alpha2_ex;
Fc = y(:,5);

% Nose and tail
nose = [-l2*sin(alpha2) l2*cos(alpha2)];
tail = [l1*sin(alpha1) -l1*cos(alpha1)];

fig = figure;
hold on
missile = plot(zeros(3,1),zeros(3,1),'LineWidth',2);
perturbation = quiver(0,0,0,0,'MaxHeadSize',10);
control = quiver(0,0,0,0,'MaxHeadSize',10);
width = (l1+l2)*1.5;
axis equal
xlim([-width/2 width/2])
ylim([-width/2 width/2])

scale = width*0.45/max(Fc);

aviobj = VideoWriter('sim_solo.avi');
aviobj.FrameRate = 24;
open(aviobj);

for k = 1:length(time)
    set(missile,'XData',[nose(k,1); 0; tail(k,1)],'YData',[nose(k,2); 0; tail(k,2)]);
    set(perturbation,'XData',nose(k,1),'YData',nose(k,2),'UData',-Fin(k)*scale);
    set(control,'XData',tail(k,1),'YData',tail(k,2),'UData',-Fc(k)*scale);
    drawnow
    
    % Record
    F = getframe(fig);
    writeVideo(aviobj,F);
end
close(aviobj);