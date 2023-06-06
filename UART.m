
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% В данном ПО представлен интерфейс для работы с радиодальномером (FM24).
% Функция SendDeviation отвечает за преобразование желаемой пользователем
% девиацией (измеряется в кГц) в напряжение (мВ) для заданной формы пилы 
% (RAMP1 - симметричная, RAMP2 - несимметричная).
% Пример: SendDeviation(500e3, coeff, RAMP1,  s);  (500e3 = 500МГц)
% coeff - задает чувствительности настройки VCO (задает пользователь во 
% время выполнения программы).
% Функция SendCommand может отправлять команды:  
% 1) Остановка ADC (STOP) 
% 2) Рестар контроллера (RESET)
% 3) Отпарвка и получение тестовой команды, т.е. эхо (TEST)
% 4) Задание формы пилы (RAMP1 или RAMP2).
% Пример: SendCommand(RESET, s).
%
% Перед началом работы пользователь должен определить параметры:
% 1) period_Number_stm32 - число периодов DAC, которое подается на stm32.
% Возможный диапазон от 1 до 4 включительно.
% 2) period_Number_pc - сколько раз ПК будет собирать данные размером 
% period_Number_stm32. Минимум значнте должно быть 1, 
% по максимуму неограничена.
% 3) SIZE_ADC - Размер буффера ADC. Должен совпадать с параметром на stm32
% 4) ADC_PER_DAC - Число периодов сбора данных, приходящихся на 1 период
% DAC. Должно совпадать с параметром на stm32.
% 5) ADC_FREQ - частота сбора данных ADC. Должна совпадать с параметром на 
% stm32.
% 
% На графиках будет отображаться period_Number_stm32*period_Number_pc
% периодов сигнала.
% Все комманды отправляются от 1 до 4 байт включительно.
% Ниже приведена таблица соответсвия по командам.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% bytes   [0]          [1]            [2]          [3]
% START    1   period_Number_stm32<=14 -            -    
% STOP     2            -              -            -
% RESET    3            -              -            -
% TEST     4            -              -            -
% RAMP1    5            -              -            -
% RAMP2    6            -              -            -
% AMPL     7      RAMP1 or RAMP2    Amplitude(0 - 4095) - 12bite ()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
START = 1; STOP = 2; RESET = 3; TEST = 4; RAMP1 = 5; RAMP2 = 6; AMPL = 7;
period_Number_stm32 = 4;
period_Number_pc = 1;
SIZE_ADC = 128;
ADC_PER_DAC = 4;
ADC_FREQ = 576e3;

s = serialport("COM6", 3e6, 'Timeout', 1);
pause(1);
count = 0;
All_Channel_new = [];
pause(0.5);
coeff = input('Введите значение чувствительности настройки VCO: ');
SendDeviation(250e3, coeff, RAMP1, s);
size_uint32 = 4; % 4 bytes
pause(0.5);
while(true)
    if count < 25000
        flush(s,"input");
        num = uint32(0);
        SendStart(period_Number_stm32, s);
        size_m = period_Number_stm32*SIZE_ADC*ADC_PER_DAC + 1;
        data = uint32([]); % Создаем пустой массив для данных
        message_size = (size_m - 1)*size_uint32; % размер в байтах
        preamble = bitshift(uint32(1), 0) + uint32(bitshift(uint32(period_Number_stm32), 8)) + uint32(bitshift(uint32(message_size), 16));
        while(length(data) < size_m) 
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
            fs = ADC_FREQ; % Частота дискретизации
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
            xlim([0, 20*10^3]) % Ограничение оси x до 20*10^3
            grid on;

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
            xlim([0, 20*10^3]) % Ограничение оси x до 20*10^3
            grid on;
            All_Channel_new = [];
            data = [];
            flush(s);
        end
    end
end
