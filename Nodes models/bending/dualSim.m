function dualSim(nodes1,nodes2,Fdist,Fcontrol,i_F,time,slowmo,n_nodes)


scale = 1;
ymax = max(abs([nodes1(:);nodes2(:)]));
scaleF = ymax/max(Fdist(:));
x = linspace(0,4.9,n_nodes);
x_F = 4.9*(i_F-1)/(n_nodes-1);
fig = figure;
hold on
display1 = plot(x,nodes1(1,:));
display2 = plot(x,nodes2(1,:));
disturbance = quiver([0;0],[nodes1(1,1);nodes2(1,1)],[0;0],scaleF*[Fdist(1);Fdist(1)],0);
control = quiver(x_F(2:3)',[nodes2(1,i_F(2));nodes2(1,i_F(3))],[0;0],scaleF*[Fcontrol(1,1);Fcontrol(1,2)],0);
textTime = text(0,-scale*ymax*9/10,'');
ylim(scale*[-ymax,ymax]);
dt = time(2)-time(1);

%
aviobj = VideoWriter('sim.avi');
aviobj.Quality = 25;
aviobj.FrameRate = 24;
open(aviobj);


for t = 1:length(time)
    pause(dt*slowmo)
    set(display1,'Ydata',nodes1(t,:));
    set(display2,'Ydata',nodes2(t,:));
    set(disturbance,'Ydata',[nodes1(t,1);nodes2(t,1)],'Vdata',scaleF*[Fdist(t);Fdist(t)]);
    set(control,'Ydata',[nodes2(t,i_F(2));nodes2(t,i_F(3))],'Vdata',scaleF*[Fcontrol(t,1);Fcontrol(t,2)]);
    set(textTime,'String',strcat('Time = ',num2str(time(t)*1000,3),' ms'));
    drawnow
    time(t)
    % Record
    %F = getframe(fig);
    %writeVideo(aviobj,F);
end
close(aviobj);