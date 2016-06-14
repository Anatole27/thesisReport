function displaySimTraj(Z0,X0,gamma,alph0,V0,xCG,thetaT0,thetaT,T0,z,x,theta,w,u,w_nodes,time,slowmo,n_nodes)

vibe_scale = 10^3;

w_nodes = vibe_scale*w_nodes;
theta0 = alph0+gamma;

[x_body,y_body] = getXYbody(Z0,X0,V0,gamma,z,x,theta0,theta,w_nodes,4.9,xCG,n_nodes,time);
[vx,vy] = getSpeed(theta0,theta,gamma,V0,w,u);

window = figure; hold on;
missile = plot(x_body(1,:),y_body(1,:));
thrust = quiver(x_body(1,1),y_body(1,1),cos(thetaT0+thetaT(1)),sin(thetaT0+thetaT(1)));
incidence = quiver(x_body(1,floor(end/2)),y_body(1,floor(end/2)),vx(1),vy(1));
%axis equal
dt = time(2)-time(1);

for t = 1:length(time)
    pause(0.0004)
    set(missile,'Xdata',x_body(t,:),'Ydata',y_body(t,:));
    set(trajPlot,'Xdata',traj(1:t,1),'Ydata',traj(1:t,2));
    set(thrust,'Xdata',x_body(t,1),'Ydata',y_body(t,1), ...
        'Udata',cos(thetaT0+thetaT(t)+alph0+gamma+theta(t)),'Vdata',sin(thetaT0+thetaT(t)+alph0+gamma+theta(t)));
    set(incidence,'Xdata',x_body(t,floor(end/2)),'Ydata',y_body(t,floor(end/2)), ...
        'Udata',vx(t),'Vdata',vy(t));
    drawnow
    %time(t)
end

end

function [x_body,y_body] = getXYbody(Z0,X0,V0,gamma,z,x,theta0,theta,w_nodes,L,xCG,n_nodes,time)

steps = length(theta);


x_frame = linspace(-xCG,L-xCG,n_nodes);
x_body = cos(theta0+theta)*x_frame - diag(sin(theta0+theta))*w_nodes + (X0 + x + time*V0*cos(gamma))*ones(1,n_nodes);
y_body = diag(cos(theta0+theta))*w_nodes + sin(theta0+theta)*x_frame - (Z0 + z + time*V0*sin(gamma))*ones(1,n_nodes);

end

function [vx,vy] = getSpeed(theta0,theta,gamm0,V0,w,u)

vx = V0*cos(gamm0) + u.*cos(theta0+theta) + w.*sin(theta0 + theta);
vy = V0*sin(gamm0) + u.*sin(theta0+theta) - w.*cos(theta0 + theta);

% Normalize
for i = 1:length(vx)
    v0 = sqrt(vx(i)^2+vy(i)^2);
    vx(i) = vx(i)/v0;
    vy(i) = vy(i)/v0;
end

end