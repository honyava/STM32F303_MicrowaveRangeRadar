function SendStart(period_Number, s) %max 14 period number
    if ((period_Number < 15) && (period_Number >= 0))
        command = bitshift(uint16(1), 0) + uint16(bitshift(uint16(period_Number), 8));
        s.write(command,"uint32");
    else
        printf("You should write period Number <= 14")
        s.write(0x03,"uint32"); % send reset
    end


end