clear s;
% A = linspace(0, 6, 7);
dt = 0.01;
fo=1;
tt = 0:dt:1;

A = 2*pi*sin(2*pi*fo*tt);

s = serialport("COM4", 115200);
while(1)
    for i=1:1:length(A)
        writeline(s,string(A(i)));
        data = readline(s);
        disp(data);
        pause(dt)
    end
end
