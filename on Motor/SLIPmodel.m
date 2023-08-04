function [th1, th2] = SLIPmodel(dt)

clc
clear
close all
format long

%% initial parameters
robot.g = 9.81;
robot.ground = 0;
robot.l = 0.4;
robot.m = 2;
robot.control.k = 400;
robot.control.theta  = 5*(pi/180);
x0dot = 0.5; 
y0 = 0.6;
z0 = [x0dot y0];

steps = 5;
fps = 30;
dt = 5;

robot.check = 0;

robot.l1 = 0.3;
robot.l2 = 0.3;

%% Root finding
options = optimset('TolFun',1e-10,'TolX',1e-10,'Display','iter');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,robot);
if exitflag == 1
    disp('Fixed point:');disp(zstar);
else
    error('Root finder not converged, change guess or change system parameters')
end
 
%% Stability, using eigenvalues
J=partialder(@onestep,zstar,robot);
disp('EigenValues for linearized map are');
eig(J)
 
%% Get data for all the steps
[z,t] = onestep(zstar,robot,steps,fps);
result.x_pos = z(:,1);
result.x_vel = z(:,2);

result.y_pos = z(:,3);
result.y_vel = z(:,4);

result.foot_x = z(:,5);
result.foot_y = z(:,6);

result.len = sqrt((result.x_pos-result.foot_x).^2 + (result.y_pos-result.foot_y).^2);
result.theta1 = 90-acosd((robot.l1^2-robot.l2^2+result.len.^2)./(2*result.len*robot.l1));
result.theta2 = 180-acosd((robot.l1^2+robot.l2^2-result.len.^2)/(2*robot.l1*robot.l2));

th1 = result.theta1;
th2 = result.theta2;

%% Test
%manipulator(robot, result.theta1, result.theta2)
%animate(t,z,robot,steps,fps);

%% functions
%==================================================================
function zdiff=fixedpt(z0,robot)
zdiff=onestep(z0,robot)-z0; 
end
%===================================================================
function J=partialder(FUN,z,robot)
pert=1e-5;
n = length(z);
J = zeros(n,n);

for i=1:n
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    J(:,i)=(feval(FUN,ztemp1,robot)-feval(FUN,ztemp2,robot)) ;
end
J=J/(2*pert);
end
%===================================================================
function [z,t]=onestep(z0,robot,steps,fps)
flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0;
    steps = 1;
    fps = 50;
end

x0 = 0; x0dot = z0(1);  
y0 = z0(2); y0dot = 0;
z0 = [x0 x0dot y0 y0dot];
t0 = 0; 

t_ode = t0;
z_ode = [z0 ...
         x0+robot.l*sin(robot.control.theta) ...
         y0-robot.l*cos(robot.control.theta)];

for i=1:steps
    
    %-------------------- apex to ground --------------------------%
    robot.check = 1;
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@condition);
    tspan = linspace(t0,t0+dt,dt*10);
    [t_temp1,z_temp1]=ode45(@action,tspan,z0,options1,robot);
    [t_temp1,z_temp1] = loco_interpolate(t_temp1,z_temp1,10*fps);
 
    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp1(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp1(end);
    z0(1:4) = z_temp1(end,1:4);
    x_com = z0(1);
    z0(1) = -robot.l*sin(robot.control.theta);
    x_foot = x_com + robot.l*sin(robot.control.theta); 
    y_foot = robot.ground;

    %----------------------- stance phase ----------------------------%
    robot.check = 2;
    options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@condition);
    tspan = linspace(t0,t0+dt,dt*10);
    [t_temp2,z_temp2]=ode45(@action,tspan,z0,options2,robot);
    [t_temp2,z_temp2] = loco_interpolate(t_temp2,z_temp2,10*fps);
    
    z_temp2(:,1) = z_temp2(:,1) + x_foot; %absolute x co-ordinate
    
    z_temp2 = [z_temp2 ...
               x_foot*ones(length(z_temp2),1) ...
               y_foot*ones(length(z_temp2),1) ...
               ];
    t0 = t_temp2(end);
    z0(1:4) = z_temp2(end,1:4);
    
    %----------------------- ground to apex --------------------------%
    robot.check = 3;
    options3 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@condition);
    tspan = linspace(t0,t0+dt,dt*10);
    [t_temp3,z_temp3]=ode45(@action,tspan,z0,options3,robot);
    [t_temp3,z_temp3] = loco_interpolate(t_temp3,z_temp3,10*fps);

     z_temp3 = [z_temp3 ...
               z_temp3(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp3(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp3(end);
    z0(1:4) = z_temp3(end,1:4);

    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end);  t_temp3(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:); z_temp3(2:end,:)];

    % position, velocity data
    [t_interp,z_interp] = loco_interpolate(t_ode,z_ode,fps);
end

z = [z0(2) z0(3)];

if flag==1
   z=z_ode;
   t=t_ode;
end

end
%===================================================================
function zdot = action(t,z,robot)

% flight
if robot.check == 1 | robot.check == 3
    zdot = [z(2) 0 z(4) -robot.g]';

% stance
else
    x = z(1); y = z(3);
    l = sqrt(x^2+y^2);
    F_spring = robot.control.k*(robot.l-l);
    Fx_spring =  F_spring*(x/l);
    Fy_spring = F_spring*(y/l);
    Fy_gravity = robot.m*robot.g;
    xddot = (1/robot.m)*(Fx_spring);
    yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);
    zdot = [z(2) xddot z(4) yddot]';

end

end
%===================================================================
function [gstop, isterminal,direction]=condition(t,z,robot)

% contact
if (robot.check == 1)
    gstop = z(3) - robot.l*cos(robot.control.theta); %position is 0;
    direction = -1;
    isterminal = 1;

% release
elseif (robot.check == 2)
    l = sqrt(z(1)^2+z(3)^2);
    gstop = l-robot.l;
    direction = 1;
    isterminal = 1;

% apex
else
    gstop = z(4) - 0; %ydot is 0;
    direction = 0;
    isterminal = 1;

end
end
%===================================================================
function animate(t_all,z_all,robot,steps,fps)

[t_interp,z_interp] = loco_interpolate(t_all,z_all,fps);

[mm,nn] = size(z_interp);
min_xh = min(z_interp(:,1)); max_xh = max(z_interp(:,1)); 
dist_travelled = max_xh - min_xh;
camera_rate = dist_travelled/mm;

window_xmin = -1.0*robot.l; window_xmax = robot.l;
window_ymin = -0.1; window_ymax = 1.9*robot.l;

axis('equal')
axis([window_xmin window_xmax window_ymin window_ymax])
axis off
set(gcf,'Color',[1,1,1])

% animate
figure(1);
for i=1:length(t_interp)
   
    plot(z_interp(i,1),z_interp(i,3),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); %com
    line([z_interp(i,1) z_interp(i,5)],[z_interp(i,3) z_interp(i,6)],'Linewidth',4,'Color',[0 0.8 0]); %leg

    axis([0.0 1.3 0.0 0.7])

    pause(0.05);
end
end
%===================================================================
function [t_interp,z_interp] = loco_interpolate(t_all,z_all,fps)
[m,n] = size(z_all);
t_interp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));

for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),t_interp);
end
t_interp = t_interp';
end
%===================================================================
function manipulator(robot, th1,th2)
ct = 1;
for i = 1:length(th1)
    theta1 = th1(i);
    theta2 = th2(i);

    % Initial point of link 1
    x0 = 0;
    y0 = 0;
    
    % Final point of link 1 & Initial point of link 2
    x1 = robot.l1*cosd(theta1);
    y1 = robot.l2*sind(theta1);

    % Final point of link 2
    x2 = x1 + robot.l2*cosd(theta1+theta2);
    y2 = y1 + robot.l2*sind(theta1+theta2);
     
    txt1 = ['θ1 = ' , num2str(theta1) , ' deg'];
    txt2 = ['θ2 = ' , num2str(theta2) , ' deg'];
     
    plot([x0 x1],[y0 y1],[x1 x2],[y1 y2],'linewidth',5)
    xlabel('X-axis (m)')
    ylabel('Y-axis (m)')
    text(x0,y0,txt1,'VerticalAlignment','top')
    text(x1,y1,txt2,'VerticalAlignment','top')
     
grid on
axis([-1 2 -0.5 2])
pause(0.01)
M(ct) = getframe(gcf);
ct = ct+1;
end

end

end