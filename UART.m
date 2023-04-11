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
% Example : s.write(0x0E060007,"uint32"); % ampl = 3590 

V_my = [64 128 256 400 550 700 850 1000 1200 1400 1600 1800 2000 2200 2400 2600 3000 3250 3500 4000];
V_my = V_my*3300/4095; % преобразование значений АЦП в милливольты
f_div = [34 68 132 200 264 332 436 500 608 712 856 960 1068 1184 1294 1410 1662 1824 2004 2436]*10^6;
V_tune = 720*10^6; % МГц/В
f_datasheet = V_tune*V_my/1000;

figure(); % создание новой фигуры
plot(f_div, V_my, '-o'); % первый график
xlabel('Девиация (ГГц)');
ylabel('Напряжение (мВ)');
title('Зависимость напряжения от девиации частоты');
grid on;

hold on; % добавление второго графика на текущую фигуру
plot(f_datasheet, V_my, '-s'); % второй график
legend('Измеренные данные', 'Данные из даташита');

s = serialport("COM3", 3e6, 'Timeout', 1);
s.write(0x09060607,"uint32"); % ampl = 3590 
  pause(1);
 %s.write(6,"uint32"); % RAMP2
% pause(1);
% s.write(0x0401,"uint32"); % START + 4 periods 
count = 0;
All_Channel_new = [];
while(true)
    if count < 100
        num = uint32(0);
        s.write(0x0401,"uint32");
        pause(0.1);
        data = uint32(read(s,4*128*4 + 1,"uint32"));
        count = count + 1;
    else
        break;
    end
%if (data(1) == 134217985)
    pause(0.5);
    for i = 2:length(data)
        num = uint32(data(i)); % исходное 32-битное число
        mask = uint32(hex2dec('FFFF')); % маска для 16-битного числа
        Channel1(i - 1) = bitand(num, mask); % получить младшие 16 бит
        Channel2(i - 1) = bitand(bitshift(num, -16), mask); % получить старшие 16 бит
    end
    All_Channel = cat(1,Channel1, Channel2);
    num2 = uint32(data(1)); % 32-битное число
    first_num = bitshift(num2, -16);
    second_num = bitand(bitshift(num2, -8), hex2dec('FF'));
    third_num = bitand(num2, hex2dec('FF'));
    preamble(count,1:3) = [first_num, second_num, third_num];
%     All_Channel(3, (count - 1)*128*4 + 1) = first_num;
%     All_Channel(4, (count - 1)*128*4 + 1) = second_num;
%     All_Channel(5, (count - 1)*128*4 + 1) = third_num;
     All_Channel_new = cat(2,All_Channel_new, All_Channel);
%     All_Channel_new(3, (count - 1)*128*4 + 1) = first_num;
%     All_Channel_new(4, (count - 1)*128*4 + 1) = second_num;
%     All_Channel_new(5, (count - 1)*128*4 + 1) = third_num;
end
%end

