%close all;
clear all;
clc;


%% Параметры акселерометра
sample_time_acc = 0.1; % период дискретизации
sample_freq = 10; % период дискретизации


%% filter (butter)
fc_b = 100;
n = 4; %4


%% end of name file
end_name = '_f100Hz.csv';

file_name = "gyro_31_07.csv";
file_read = readmatrix(file_name);
        
Ax = file_read(:,2)';
Ay = file_read(:,3)';
Az = file_read(:,4)';
time = sample_time_acc * (0 : 1 : length(Ax)-1)';

%{
for i = 1 : length(Ax)
    Ax_ = Ax(i);
    Ay_ = Ay(i);
    Ax(i) = (Ax_ + Ay_)*sqrt(2)/2;
    Ay(i) = (Ax_ - Ay_)*sqrt(2)/2;
end
%}

%% filter (butter)
fs_b = 10;
plot_psd_on = 1
  
if plot_psd_on
    % filt atfer interpolation
    % lowpass
    fs_0 = sample_freq;
    f_pass_0 = 0.25*sample_freq;
    f_stop_0 = 0.45*sample_freq;
    Ax_interpol_0 = lowpass(Ax,f_pass_0,fs_0,'ImpulseResponse','fir','Steepness',0.9);
    Ay_interpol_0 = lowpass(Ay,f_pass_0,fs_0,'ImpulseResponse','fir','Steepness',0.9);
    Az_interpol_0 = lowpass(Ay,f_pass_0,fs_0,'ImpulseResponse','fir','Steepness',0.9);
    
    
    plot_psd4(Ax,Ay,Ay,fs_0);
    legend('wx','wy','wz');
    
    
    plot_psd4(Ax_interpol_0,Ay_interpol_0,Ay_interpol_0,f_pass_0);
    legend('Ax_interpol_0','Ay_interpol_0','Az_interpol_0');
        
        
        
    % filter designer
    fs_1 = sample_freq;
    f_pass_1 = 0.25*sample_freq;
    f_stop_1 = 0.45*sample_freq;
    d = designfilt('lowpassfir', ...        % Response type
        'PassbandFrequency',f_pass_1, ...     % Frequency constraints
        'StopbandFrequency',f_stop_1, ...
        'PassbandRipple',0.5, ...          % Magnitude constraints
        'StopbandAttenuation',50, ...
        'SampleRate',fs_1);            % Sample rate
     fvtool(d);
     Ax_interpol_1 = conv(Ax,d.Coefficients,'same');
     Ax_interpol_1 = Ax;
     Ay_interpol_1 = conv(Ay,d.Coefficients,'same');
     Ay_interpol_1 = Ay;
end
       
%% filter (100Hz)
% butter filt
[b,a] = butter(n,fc_b/(fs_b/2));

Ax_filt400 = filter(b,a,Ax);
Ay_filt400 = filter(b,a,Ay_resample);


time_filt400 = time_resample;

Ax_1 = filter(b,a,Ax_resample);
Ax_2 = filter(b,a,Ax_1(end:-1:1));
Ax_2 = Ax_2(end:-1:1);


Ax_filt400_1 = filter(b,a,Ax_resample);
Ax_filt400_1_shift = Ax_filt400_1(73:end);
time_filt400_1_shift = time_filt400(1:end-72);


%% smooth
Ax_smooth = movmean(Ax_filt400,point_smooth);
Ay_smooth = movmean(Ay_filt400,point_smooth);
time_smooth = time_filt400;
if side_on
    left_smooth = movmean(left_filt400,point_smooth);
    right_smooth = movmean(right_filt400,point_smooth);
    
    left_y_smooth = movmean(left_y_filt400,point_smooth);
    right_y_smooth = movmean(right_y_filt400,point_smooth);
    
    left_z_smooth = movmean(left_z_filt400,point_smooth);
    right_z_smooth = movmean(right_z_filt400,point_smooth);
end
      
                
        
%% plot (ref/filt_acc)
coef_ = 9.81;
if plot_ref == 1
    figure;
    if side_on
        N = 4;
    else
        N = 2;
    end
    
    subplot(N,1,1);
    name = strrep(file_name1,'_',' ');
    title(name);
    
    hold on;
    plot(time,Ax*coef_,'b');
    plot(time_filt,Ax_filt*coef_,'r');
    hold off;
    
    xlabel('time, ms');
    ylabel('Ax, g');
    legend({'Исходные','Фильтрованные'},'Location','southeast');
    
    subplot(N,1,2);
    
    hold on;
    plot(time,Ay*coef_,'b');
    plot(time_filt,Ay_filt*coef_,'r');
    hold off;
    
    xlabel('time, ms');
    ylabel('Ay, g');
    legend({'Исходные','Фильтрованные'},'Location','southeast');
    
    if side_on
        subplot(N,1,3);
        
        hold on;
        plot(time,left*coef_,'b');
        plot(time_filt,left_filt*coef_,'r');
        hold off;
        
        xlabel('time, ms');
        ylabel('left, g');
        legend({'Исходные','Фильтрованные'},'Location','southeast');
        
        subplot(N,1,4);
        
        hold on;
        plot(time,right*coef_,'b');
        plot(time_filt,right_filt*coef_,'r');
        hold off;
        
        xlabel('time, ms');
        ylabel('right, g');
        legend({'Исходные','Фильтрованные'},'Location','southeast');
    end
    
end