clear;
% bytes   [0]          [1]            [2]          [3]
% START    1    period_Number_stm32<=14      -            -    
% STOP     2            -              -            -
% RESET    3            -              -            -
% TEST     4            -              -            -
% RAMP1    5            -              -            -
% RAMP2    6            -              -            -
% AMPL     7          5 or 6            Amplitude(0 - 4095) - 12bite ()


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
START = 1; STOP = 2; RESET = 3; TEST = 4; RAMP1 = 5; RAMP2 = 6; AMPL = 7;
%SendDeviation(500e3, RAMP1, s); %deviation - kHz (in function 500e3 = 500MHz)
%SendCommand(command, s) %only for STOP_RESET_TEST_RAMP1_RAMP2
period_Number_stm32 = 4;  % max period_Number_stm32_stm32 = 4
period_Number_pc = 2; % can be any number

% on graph will be period_Number_stm32*period_Number_pc PERIODS of signal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


s = serialport("COM6", 3e6, 'Timeout', 1);
pause(1);
count = 0;
All_Channel_new = [];
pause(0.5);
SendDeviation(250e3, RAMP1, s);
pause(0.5);
while(true)
    if count < 25000
        flush(s,"input");
        num = uint32(0);
        SendStart(period_Number_stm32, s);
        size_m = period_Number_stm32*128*4 + 1;
        data = uint32([]); % Создаем пустой массив для данных
%         data = uint32(read(s, size_m,"uint32"));
        message_size = (size_m - 1)*4; % in bytes
        preamble = bitshift(uint32(1), 0) + uint32(bitshift(uint32(period_Number_stm32), 8)) + uint32(bitshift(uint32(message_size), 16));

        while(length(data) < size_m) % Ждем, пока длина данных станет больше или равной 4*128*4
            new_data = uint32(read(s, size_m - length(data), 'uint32'));
            % Добавляем новые данные в конец массива data
            data = [data; new_data];
            if isempty(new_data)
                pause(0.1);
            end
        end

        
        count = count + 1;
    else
        break;
    end

    if (data(1) == preamble)
        
        for i = 2:length(data)
            num = uint32(data(i)); % исходное 32-битное число
            mask = uint32(hex2dec('FFFF')); % маска для 16-битного числа
            Channel1(i - 1) = bitand(num, mask); % получить младшие 16 бит
            Channel2(i - 1) = bitand(bitshift(num, -16), mask); % получить старшие 16 бит
        end
        All_Channel = cat(1,Channel1, Channel2);    
        All_Channel_new = cat(2,All_Channel_new, All_Channel);

        if (mod(count, period_Number_pc) == 0)
            fs = 576e3; % Частота дискретизации
            t = (0:numel(All_Channel_new(1,:))-1) * 1/fs * 1000; % Преобразование в миллисекунды
        
            subplot(4, 1, 1) % Создание первого графика
            plot(t,All_Channel_new(1,:)) % График отсчетов Channel1
            title("Channel IFI")
            xlabel("Time (ms)"); % Изменяем подпись оси x
            ylabel("Amplitude")
            grid on;
            
            Y = fft(All_Channel_new(1,:));
            L = length(All_Channel_new(1,:));
            P2 = abs(Y/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);
            f = (0:L/2)*(1/(L/2))*(fs/2);
            
            subplot(4, 1, 2) % Создание второго графика
            plot(f,P1) 
            title("Single-Sided Amplitude Spectrum of IFI")
            xlabel("f (Hz)")
            ylabel("|P1(f)|")
            xlim([0, 20*10^3]) % Ограничение оси x до 10^5
            grid on;
            % drawnow;

            subplot(4, 1, 3) % Создание третьего графика
            plot(t,All_Channel_new(2,:)) % График отсчетов Channel1
            title("Channel IFQ")
            xlabel("Time (ms)"); % Изменяем подпись оси x
            ylabel("Amplitude")
            grid on;

            Y = fft(All_Channel_new(2,:));
            L = length(All_Channel_new(2,:));
            P2 = abs(Y/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);
            f = (0:L/2)*(1/(L/2))*(fs/2);

            subplot(4, 1, 4) % Создание четвертого графика
            plot(f,P1) 
            title("Single-Sided Amplitude Spectrum of IFQ")
            xlabel("f (Hz)")
            ylabel("|P1(f)|")
            xlim([0, 20*10^3]) % Ограничение оси x до 10^5
            grid on;
            All_Channel_new = [];
            data = [];
            flush(s);
        end
    end
end
