clear;
% bytes   [0]        [1]            [2]          [3]
% START    1     Number_period       -            -
% STOP     2          0              -            -
% RESET    3          0              -            -
% TEST     4          0              -            -
% RAMP1    5          0              -            -
% RAMP2    6          0              -            -
% AMPL     7          0            Amplitude(0 - 4095) - 12bite

% Example : s.write(0x0E060007,"uint32"); % ampl = 3590 


s = serialport("COM5", 3e6, 'Timeout', 0.5);
s.write(0x0E060007,"uint32"); % ampl = 3590 
pause(1);
s.write(6,"uint32"); % RAMP2
pause(1);
s.write(0x0901,"uint32"); % START + 9 periods 
data = read(s,384*100,"uint32");

%s.write("STAR","char"); %% STAR - START, STOP - STOP, TEST - TEST, RESE - RESET
%data = read(s,384*100,"uint32");

