clc; clear;
datapath = "motor_data\cleaned\";

data={};
files = ls(datapath);
files = string(files);
for i = 1 : size(files, 1)
    if startsWith(files(i), "m")
        idx = size(data, 1) + 1;
        tempData = ...
            readtable(strtrim(datapath+files(i,:)));
        
        %tempData{:,5}(end) = 0;
        data{idx, 1} = tempData{:,2}; % time
        data{idx, 2} = tempData{:,3}; % pwm
        data{idx, 3} = tempData{:,4}; % counts
        data{idx, 4} = tempData{:,5}; % speed
    end
end

% Read Data 
% 1)Time, 2)Time_1"sample step" 3)I/P 4)O/P Counter 5)Speed
%motor_data = readtable(datapath);
%time = motor_data{:,2};
%input = motor_data{:,3};
%speed = motor_data{:,5};
%speed(end) = 0;



%figure; plot(time, input); title('Unit Step Input');
%figure; plot(time, speed); title('Output Speed');

%sys = iddata(speed, input, 0.005);
%poles = 2; zeros = 1; ts = 0.005;
%txy = tfest(sys, poles, zeros, 0, 'Ts', 0.005);
%figure; step(txy); title('Discrete System');
%cont_sys = d2c(txy, 'zoh');
