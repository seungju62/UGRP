clear s;

a = num2str(123456789);

s = serialport("COM3",115200);
lst=[];
for num=0:1:15
    data = readline(s);
%    lst = append(lst, data);
    disp(data);
    writeline(s,a);
    pause(1);
end
% start point : 0
% finish point : 10 ?