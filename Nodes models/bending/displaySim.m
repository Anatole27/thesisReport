function displaySim(nodes,time,slowmo,n_nodes)
scale = 1;
x = linspace(0,4.9,n_nodes);
window = plot(x,nodes(1,:));
ymax = max(abs(nodes(:)));
ylim(scale*[-ymax,ymax]);
dt = time(2)-time(1);
for t = 1:length(time)
    pause(dt*slowmo)
    set(window,'Ydata',nodes(t,:));
    drawnow
    time(t)
end