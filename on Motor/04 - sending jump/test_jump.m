clear s;
dt = 0.005;
delay = 3;

fo=1;
tt = 0:dt:1;

[theta1, theta2] = JumpingModel();

format short

theta1 = string(round(theta1, 4));
theta2 = string(round(theta2, 4));

for i=1:1:length(theta1)
    theta1(i)=sprintf('%0.4f',theta1(i));
    theta2(i)=sprintf('%0.4f',theta2(i));
end

new_theta = theta1+theta2;

s = serialport("COM5", 115200);
while(1)
    for i=1:1:length(theta1)
        writeline(s,new_theta(i));
        % writeline(s,string(A(i)));
        data = readline(s);
        disp(data);
        pause(dt)
    end
    pause(delay)
end