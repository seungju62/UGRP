clear s;
dt = 0.01;
fo=1;
tt = 0:dt:1;

% A = 2*pi*sin(2*pi*fo*tt);

[theta1, theta2] = SLIPmodel();


s = serialport("COM4", 115200);
while(1)
    for i=1:1:length(theta1)
        writeline(s,string(theta1(i)));
        data = readline(s);
        disp(data);
        pause(dt)
    end
end

