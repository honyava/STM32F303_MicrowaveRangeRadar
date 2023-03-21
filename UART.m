clear;

% T = 1*(1/50);
% 
% fs = 1000*6.4;
% t = 0:1/fs:T-1/fs;
% 
% x = sawtooth(2*pi*50*t);
% x = (x + 1)/2;
% x = x*4095;
% x = round(x);

s = serialport("COM5", 3e6, 'Timeout', 0.5);

s.write(0x0E060901,"uint32");
data = read(s,384*100,"uint32");

%s.write("STAR","char"); %% STAR - START, STOP - STOP, TEST - TEST, RESE - RESET
%data = read(s,384*100,"uint32");

