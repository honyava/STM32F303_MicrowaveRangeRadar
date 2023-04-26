function SendCommand(command, s) %only for STOP_RESET_TEST_RAMP1_RAMP2
    s.write(command,"uint32");
end