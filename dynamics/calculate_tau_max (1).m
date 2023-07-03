clc; clear all; close all;

%% initial value
l1 = 0.20;
l2 = 0.20;
theta1 = [];
theta2 = [];

k = (50:50:1000);
m = 1;
g = 10;
x = [];

%% calculate tau_max

body_height = [0 0.076425272455525 0.156950494362335 0.200000547156717 0.227336010988913 0.246534271541896 0.260829982045613 0.271982922856677 0.280980140892768 0.288416081658687 0.294621982460507 0.300001357287313 0.304676139028268 0.308784232609930 0.312514533746488 0.315694729146179 0.318636123326106 0.321397396703802 0.323738641540839 0.326069549568129];
%body_height(1)의 값은 k=50일 때 -1.477606797749992e+02이 나오기 때문에 임의로 0으로 두었음.
%(0.4-body_height)이 delta_x값임.

tau_max = [];
tau_max_i = [];
tau_max_x = [];
tau_max_y = [];


for i = 1:1:length(k)
    
    theta1(i) = asin(body_height(i)/0.4)*pi/180;
    theta2(i) = (180-2*theta1(i))*pi/180;
    
    J = [   l1*sin(theta1(i)) + l2*sin(theta1(i) + theta2(i))    l2*sin(theta1(i) + theta2(i));
           -l1*cos(theta1(i)) - l2*cos(theta1(i) + theta2(i))    -l2*cos(theta1(i) + theta2(i))];
   
    x(i) = 0.4 - body_height(i);
    Fx(i) = -k(i)*x(i)*cos(theta1(i) + theta2(i));
    Fy(i) = -k(i)*x(i)*sin(theta1(i) + theta2(i));
    
    tau_max_i = J'*[Fx(i); Fy(i)];
    tau_max_x(i) = tau_max_i(1,:);
    tau_max_y(i) = tau_max_i(2,:);
    %tau_max(i) = sqrt(tau_max_x(i)^2 + tau_max_y(i)^2);
end

disp(tau_max);



figure(1);
subplot(1,2,1)
plot(k, tau_max_x, '-o', 'LineWidth', 2);grid on;hold on;xlabel('k');ylabel('tau_max_x'); title('max tau');
subplot(1,2,2)
plot(k, tau_max_y, '-o', 'LineWidth', 2);grid on;hold on;xlabel('k');ylabel('tau_max_y'); title('max tau');

figure(2);
plot(k, body_height, '-o', 'LineWidth', 2);grid on;hold on;xlabel('k');ylabel('body_height'); title('body height');
