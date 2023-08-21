clear s;
dt = 0.1;
fo=1;
tt = 0:dt:1;

% A = 2*pi*sin(2*pi*fo*tt);

[theta1, theta2] = SLIPmodel();

format short

theta1 = string(round(theta1, 4));
theta2 = string(round(theta2, 4));

for i=1:length(theta1)
    if theta1(i)=="0"
        theta1(i)="0.0000";
        theta2(i)="0.0000";
    else
        theta1(i)=sprintf('%0.4f',theta1(i));
        theta2(i)=sprintf('%0.4f',theta2(i));
    end
end

new_theta = theta1+theta2;

s = serialport("COM5", 115200);
while(1)
    for i=1:1:length(theta1)
        writeline(s,new_theta(i));
        data = readline(s);
        disp(data);
        pause(dt)
    end
end
