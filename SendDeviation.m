function SendDeviation(deviation, coeff, RAMP, s)
    Ampl_u = round(deviation/coeff); % deviatin - kHz;  Ampl_u - mV.           620.424
    Ampl_u = round(Ampl_u/0.8059);
%     command = uint32(0);
    command = uint32(7) + bitshift(uint32(RAMP), 8) + uint32(bitshift(uint32(Ampl_u), 16));
    s.write(command,"uint32");
end