clear;
% bytes   [0]          [1]            [2]          [3]
% START    1    period_Number<=14      -            -    
% STOP     2            -              -            -
% RESET    3            -              -            -
% TEST     4            -              -            -
% RAMP1    5            -              -            -
% RAMP2    6            -              -            -
% AMPL     7            -            Amplitude(0 - 4095) - 12bite

% max period_Number = 14
% Example : s.write(0x0E060007,"uint32"); % ampl = 3590 


s = serialport("COM5", 3e6, 'Timeout', 0.5);
s.write(0x04060007,"uint32"); % ampl = 3590 
pause(1);
s.write(6,"uint32"); % RAMP2
pause(1);
s.write(0x0401,"uint32"); % START + 9 periods 
data = read(s,384*100,"uint32");


