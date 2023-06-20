function slipHopper_sim

clc
clear all
close all
format short

% initial parameters
 robot.g = 10;
 robot.ground = 0;
 robot.l = 0.4;
 robot.m = 1;
 robot.control.k = 500;
 robot.control.theta  = 0*(pi/180);
 x0dot = 0	;  
 y0 = 0.6	;

z0 = [x0dot y0];

steps = 5;
fps = 30;

%%% Root finding, Period one gait %%%%
options = optimset('TolFun',1e-10,'TolX',1e-10,'Display','iter');
[zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,robot);
if exitflag == 1
    disp('Fixed point:');disp(zstar);
    %disp('fval:');disp(fval);
else
    error('Root finder not converged, change guess or change system parameters')
end
 
% Get data for all the steps
[z,t] = onestep(zstar,robot,steps,fps);

% Animate result
disp('Animating...');
animate(t,z,robot,steps,fps);



%===================================================================
function zdiff=fixedpt(z0,robot)
zdiff=onestep(z0,robot)-z0; 

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
dt = 5;

t_ode = t0;
z_ode = [z0 ...
         x0+robot.l*sin(robot.control.theta) ...
         y0-robot.l*cos(robot.control.theta)];


for i=1:steps
    
    % ---------------------- apex to ground --------------------------
    options1 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@contact);
    tspan = linspace(t0,t0+dt,dt*10);
    [t_temp1,z_temp1]=ode45(@flight,tspan,z0,options1,robot);
    [t_temp1,z_temp1] = loco_interpolate(t_temp1,z_temp1,10*fps);

    z1=z_temp1(:,3);
%     disp('z1'); disp(z1);
    % ph1_theta1 = acosd(z1/0.6);
    % ph1_theta2 = acosd((z1.^2-0.18)/0.18);
    ph1_theta1 = round(48.2*ones(1,dt*10),2);
    ph1_theta2 = round(ph1_theta1+90,2);
%      disp('ph1_theta1 : '); disp(ph1_theta1);
%      disp('ph1_theta2 : '); disp(ph1_theta2);

    z_temp1 = [z_temp1 ...
               z_temp1(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp1(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp1(end);
    z0(1:4) = z_temp1(end,1:4);
    x_com = z0(1); %save the x position for future
    z0(1) = -robot.l*sin(robot.control.theta); %relative distance wrt contact point because of non-holonomic nature of the system
    x_foot = x_com + robot.l*sin(robot.control.theta); 
    y_foot = robot.ground;

   
    % --------------------stance phase----------------------
    tspan = linspace(t0,t0+dt,dt*10);
    options2 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@release);
    [t_temp2,z_temp2]=ode113(@stance,tspan,z0,options2,robot);
    [t_temp2,z_temp2] = loco_interpolate(t_temp2,z_temp2,10*fps);
    
    z_temp2(:,1) = z_temp2(:,1) + x_com + robot.l*sin(robot.control.theta); %absolute x co-ordinate
%     disp('z_temp2');
%     disp(z_temp2(:,3));

    z2=z_temp2(:,3);
    ph2_theta1 = round(acosd(z2/0.6),2);
    ph2_theta2 = round(ph2_theta1+90,2);
    % ph2_theta2 = round(acosd((z2.^2-0.18)/0.18),2);
%     disp('theta1 : '); disp(ph2_theta1);
%     disp('theta2 : '); disp(ph2_theta2);

    z_temp2 = [z_temp2, ...
          x_foot*ones(length(z_temp2),1) y_foot*ones(length(z_temp2),1)]; %the distal end of leg is 0 when touching the ground.
    t0 = t_temp2(end);
    z0(1:4) = z_temp2(end,1:4);

    
    % ----------------- ground to apex ------------------
    tspan = linspace(t0,t0+dt,dt*10);
    options3 = odeset('Abstol',1e-13,'Reltol',1e-13,'Events',@apex);
    [t_temp3,z_temp3]=ode113(@flight,tspan,z0,options3,robot);
    [t_temp3,z_temp3] = loco_interpolate(t_temp3,z_temp3,10*fps);

%     disp('z_temp3');
%     disp(z_temp3(:,3));

    z3=z_temp3(:,3);
    % ph3_theta1 = acosd(z3/0.6);
    % ph3_theta2 = acosd((z3.^2-0.18)/0.18);
    ph3_theta1 = round(48.2*ones(1,dt*10),2);
    ph3_theta2 = round(ph3_theta1+90,2);
%     disp('theta1 : '); disp(ph3_theta1);
%     disp('theta2 : '); disp(ph3_theta2);

     z_temp3 = [z_temp3 ...
               z_temp3(1:end,1)+robot.l*sin(robot.control.theta) ...
               z_temp3(1:end,3)-robot.l*cos(robot.control.theta) ...
              ];
    t0 = t_temp3(end);
    z0(1:4) = z_temp3(end,1:4);
    
    %%%%% Ignore time stamps for heelstrike and first integration point
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end);  t_temp3(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:); z_temp3(2:end,:)];

    theta1_all = [ph1_theta1(:) ; ph2_theta1(:); ph3_theta1(:)];
    theta2_all = [ph1_theta2(:) ; ph2_theta2(:); ph3_theta2(:)];
%     theta2_all = [ph1_theta1(end-1)+ph1_theta2(:) ; ph2_theta1(end-1)+ph2_theta2(:); ph3_theta1(end-1)+ph3_theta2(:)];

     disp('----------theta1------------');
     disp(theta1_all);
     disp('----------theta2------------');
     disp(theta2_all);
    
    manipulator(theta1_all, theta2_all);
end
z = [z0(2) z0(3)];

if flag==1
   z=z_ode;
   t=t_ode;
end

%===================================================================
function zdot=flight(t,z,robot)  
zdot = [z(2) 0 z(4) -robot.g]';

%===================================================================
function [gstop, isterminal,direction]=contact(t,z,robot)
gstop = z(3) - robot.l*cos(robot.control.theta); %position is 0;
direction = -1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function zdot=stance(t,z,robot)  
x = z(1); y = z(3); %x & y position of com wrt ground
l = sqrt(x^2+y^2);
F_spring = robot.control.k*(robot.l-l);
Fx_spring =  F_spring*(x/l);
Fy_spring = F_spring*(y/l);
Fy_gravity = robot.m*robot.g;
xddot = (1/robot.m)*(Fx_spring);
yddot = (1/robot.m)*(-Fy_gravity+Fy_spring);
zdot = [z(2) xddot z(4) yddot]';

%===================================================================
function [gstop, isterminal,direction]=release(t,z,robot)
l = sqrt(z(1)^2+z(3)^2);
gstop = l-robot.l;
direction = 1; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function [gstop, isterminal,direction]=apex(t,z,robot)
gstop = z(4) - 0; %ydot is 0;
direction = 0; %negative direction goes from + to -
isterminal = 1;  %1 is stop the integration

%===================================================================
function animate(t_all,z_all,robot,steps,fps)
%%% interpolate for animation %%
[t_interp,z_interp] = loco_interpolate(t_all,z_all,fps);

%%%%% prepare for animation %%%%%%%
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

%%%%% now animate %%%%%%%
figure(1);
for i=1:length(t_interp)
   
    plot(z_interp(i,1),z_interp(i,3),'ro','MarkerEdgeColor','r', 'MarkerFaceColor','r','MarkerSize',20); %com
    %line([-1 max(z_interp(:,1))+1],[0 0],'Linewidth',2,'Color','black'); %ground
    line([z_interp(i,1) z_interp(i,5)],[z_interp(i,3) z_interp(i,6)],'Linewidth',4,'Color',[0 0.8 0]); %leg
     
    
    %disp([z_interp(i,1),z_interp(i,3)])
    %window_xmin = window_xmin + camera_rate;
    %window_xmax = window_xmax + camera_rate;
    %axis('equal')
    %axis off
    axis([0.0 1.3 0.0 0.5])

    pause(0.05);
end

%===================================================================
function [t_interp,z_interp] = loco_interpolate(t_all,z_all,fps)
[m,n] = size(z_all);
t_interp = linspace(t_all(1),t_all(end),fps*(t_all(end)-t_all(1)));

for i=1:n
    z_interp(:,i) = interp1(t_all,z_all(:,i),t_interp);
end
t_interp = t_interp';


%===================================================================
function manipulator(th1,th2)
% Length of links (m)
l1 = 0.3;
l2 = 0.3;

ct = 1;
for i = 1:length(th1)
 theta1 = th1(i);
 theta2 = -th2(i);
 % Coordinates:
 % Initial point of link 1
 x0 = 0;
 y0 = 0;
 
 % Final point of link 1 i.e. Initial point of link 2
 x1 = l1*sind(theta1);
 y1 = l1*cosd(theta1);
 % Final point of link 2
 x2 = x1 + l2*cosd(theta2);
 y2 = y1 - l2*sind(theta2);
 
 % Assigning the parameters
txt1 = ['θ1 = ' , num2str(theta1) , ' deg'];
txt2 = ['θ2 = ' , num2str(theta2) , ' deg'];
txtend = ['x2 = ' , num2str(x2) , ' , ' , 'y2 = ' , num2str(y2)];

 % Plotting now:
 
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
