function displaySim(nodes,time,slowmo,n_nodes,record,vidName)
% Display on deformation on beam

scale = 1;
x = linspace(0,4.9,n_nodes);
window = plot(x,nodes(1,:),'k','LineWidth',2);
fig = gca;
xlabel('Abscissa x (m)','interpreter','latex')
ylabel('Lateral displacement','interpreter','latex')
ymax = max(abs(nodes(:)));
txt = text(2.2,-ymax*3/4,'');
ylim(scale*[-ymax,ymax]);
dt = time(2)-time(1);

% Optional record
if record == 1
    aviobj = VideoWriter(vidName);
    aviobj.FrameRate = 24;
    open(aviobj);
end


for t = 1:10:length(time)
    pause(dt*slowmo)
    set(window,'Ydata',nodes(t,:));
    txt.String = strcat('t = ',num2str(round(t*dt,3)*1000),' ms');
    drawnow
    time(t)
    
    
    if record == 1
        % Optional record
        F = getframe(fig);
        writeVideo(aviobj,F);
    end
end
if record == 1
    close(aviobj);
end
close all