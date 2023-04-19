clear;
% bytes   [0]          [1]            [2]          [3]
% START    1    period_Number<=14      -            -    
% STOP     2            -              -            -
% RESET    3            -              -            -
% TEST     4            -              -            -
% RAMP1    5            -              -            -
% RAMP2    6            -              -            -
% AMPL     7          5 or 6            Amplitude(0 - 4095) - 12bite ()

% max period_Number = 14
% Example : s.write(0x0E060507,"uint32"); % ampl = 3590 

% V_my = [64 128 256 400 550 700 850 1000 1200 1400 1600 1800 2000 2200 2400 2600 3000 3250 3500 4000];
% V_my = V_my*3300/4095; % преобразование значений АЦП в милливольты
% f_div = [34 68 132 200 264 332 436 500 608 712 856 960 1068 1184 1294 1410 1662 1824 2004 2436]*10^6;
% V_tune = 650*10^6; % МГц/В
% f_datasheet = V_tune*V_my/1000;
% 
% figure(); % создание новой фигуры
% plot(f_div, V_my, '-o'); % первый график
% xlabel('Девиация (ГГц)');
% ylabel('Напряжение (мВ)');
% title('Зависимость напряжения от девиации частоты');
% grid on;
% 
% hold on; % добавление второго графика на текущую фигуру
% plot(f_datasheet, V_my, '-s'); % второй график
% legend('Измеренные данные', 'Данные из даташита');

%SendDeviation(500e3, RAMP1); %deviation - kHz (in function 500e3 = 500MHz)
SendDeviation(500e3, 5);
s = serialport("COM5", 3e6, 'Timeout', 1);
%s.write(0x09060607,"uint32"); % ampl = 3590 
  pause(1);
 %s.write(6,"uint32"); % RAMP2
% pause(1);
% s.write(0x0401,"uint32"); % START + 4 periods 
count = 0;
All_Channel_new = [];
while(true)
    if count < 10
        num = uint32(0);
        s.write(0x0401,"uint32");
        pause(0.2);
        data = uint32(read(s,4*128*4 + 1,"uint32"));
        count = count + 1;
    else
        break;
    end
    
if (data(1) == 536871937)
    pause(0.5);
    for i = 2:length(data)
        num = uint32(data(i)); % исходное 32-битное число
        mask = uint32(hex2dec('FFFF')); % маска для 16-битного числа
        Channel1(i - 1) = bitand(num, mask); % получить младшие 16 бит
        Channel2(i - 1) = bitand(bitshift(num, -16), mask); % получить старшие 16 бит
    end
    All_Channel = cat(1,Channel1, Channel2);

%     All_Channel(3, (count - 1)*128*4 + 1) = first_num;
%     All_Channel(4, (count - 1)*128*4 + 1) = second_num;
%     All_Channel(5, (count - 1)*128*4 + 1) = third_num;
     All_Channel_new = cat(2,All_Channel_new, All_Channel);
%     All_Channel_new(3, (count - 1)*128*4 + 1) = first_num;
%     All_Channel_new(4, (count - 1)*128*4 + 1) = second_num;
%     All_Channel_new(5, (count - 1)*128*4 + 1) = third_num;
end
end

Y = fft(Channel1());
L = length(Channel1);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = (0:L/2)*(1/(L/2))*(576e3/2);

plot(f,P1) 
title("Single-Sided Amplitude Spectrum of S(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")
grid on;
% c = 3e8; % скорость света
% numSamples = L;
% % Определяем заданную полосу частот
% f_min = 4e6; % Минимальная частота
% f_max = 8e6; % Максимальная частота
% deltaF = (12e6 - 0) / numSamples
% % Находим индекс элемента массива P1 с максимальным значением в заданной полосе частот
% [max_val, idx] = max(P1(f >= f_min & f <= f_max));
% idx = find(P1 == max_val, 1);
% % [~,idx] = max(P1); % индекс максимального значения в спектре
% sweepBandwidth = 100e3;
% f0 = 5e6;
% range = c * idx * deltaF / (2 * f0 * numSamples); % расстояние до объекта

% c = 3e8; % скорость света в м/с
% f0 = 24.125e9; % рабочая частота в ГГц
% deltaF = 100e3; % ширина спектра сигнала в ГГц
% numSamples = 256; % количество отсчетов АЦП
% adcFullScale = 2^12; % полный диапазон АЦП в битах
% 
% [max_val, idx] = max(P1); % нахождение максимального значения и его индекса
% 
% % определение полосы частот, в которой происходит измерение
% f_range = 10e6; % ширина полосы частот в ГГц
% f_min = f0 - f_range/2;
% f_max = f0 + f_range/2;
% idx_range = idx - f_range/deltaF/2 : idx + f_range/deltaF/2;
% 
% % поиск индекса максимального значения в заданной полосе частот
% [max_val, idx] = max(P1(idx_range));
% 
% % расчет расстояния до объекта
% range = c * idx * deltaF / (2 * f0 * numSamples); % расстояние до объекта в метрах
% 
% 
% 
% % Вывод результата
% fprintf('Расстояние: %.2f м\n', range);
function SendDeviation(deviation, RAMP)
    Ampl_u = round(deviation/620.424); % deviatin - kHz;  Ampl_u - mV.
    Ampl_u = round(Ampl_u/0.8059);
    command = uint32(7) + bitshift(uint32(RAMP), 8) + uint32(bitshift(uint32(Ampl_u), 16));
%     command = uint32(strcat(0,command));
    s.write(command,"uint32");
end

