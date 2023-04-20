function SendDeviation(deviation, RAMP, s)
    Ampl_u = round(deviation/620.424); % deviatin - kHz;  Ampl_u - mV.
    Ampl_u = round(Ampl_u/0.8059);
%     command = uint32(0);
    command = uint32(7) + bitshift(uint32(RAMP), 8) + uint32(bitshift(uint32(Ampl_u), 16));
    s.write(command,"uint32");
end