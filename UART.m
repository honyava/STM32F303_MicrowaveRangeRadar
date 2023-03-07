clear;
s = serialport("COM5", 3e6, 'Timeout', 0.5);
s.write("STAR","char"); %% STAR - START, STOP - STOP, TEST - TEST, RESE - RESET
data = read(s,100000,"uint32");

