%% BYOM_Analysis.m
%
% Description:
%   Reads in speed estimates from numerical differencing (which introduces quantization noise),
%   smooths speed with a zero-phase digital lowpass IIR filter,
%   and estimates acceleration by numerically differencing the filtered speed signal.
%
% Author: Dan Lynch
% Date: Jan. 14, 2019

function BYOM_Analysis
    clear;
    close all;
    clc;
    
    % import PuTTY log
%     [log_file, log_path] = uigetfile({'*.log;'},'Select a log file');
    log_file = 'putty.log';
    log_path = [pwd,'/data/'];
    log_text = fileread([log_path,log_file]);
    log_text_as_cells = regexp(log_text, '\r\n', 'split');
    log_size = size(log_text_as_cells);
    log_len = max(log_size);
    % find the line that reads 'Setup complete!':
    [~,setup_complete_line_num] = max(~cellfun(@isempty, ...
        regexp(log_text_as_cells, 'Setup complete!')));
    % find empty line at end:
    [~,empty_ind] = max(strcmp(log_text_as_cells(setup_complete_line_num:end),''));
    empty_line_num = setup_complete_line_num + empty_ind - 1;
    
    % parse (hall state, current count) from log:
    for i = (setup_complete_line_num+2):(empty_line_num-1)
        line_str = regexp(log_text_as_cells{i},',','split');
        hall_state(i-(setup_complete_line_num + 1)) = ...
            str2double(cell2mat(regexp(line_str{1},'\d','match')));
        curr_count(i-(setup_complete_line_num + 1)) = ...
            str2double(cell2mat(line_str(2)));
    end
    
    % compute angle steps from hall state:
    theta_step = 0;
    step_time = 0;
    for i = 2:length(hall_state)
       if hall_state(i) ~= hall_state(i-1)
           theta_step = [theta_step, theta_step(end) + 30];
           step_time = [step_time, i];
       end
    end
    
    % compute smoothed angle from stepped angle:
    theta = interp1(step_time,theta_step,(0:1:length(hall_state)-1),'spline');
    
    % estimate velocity by numerically differentiating the smoothed angle
    % measurement:
    dt = 1/5000;
    omega = [0, diff(theta)/dt];

    % import data
%     [csv_file, csv_path] = uigetfile({'*.csv;'},'Select a CSV file');
%     spinup_matrix = csvread([csv_path,csv_file],2,0);
%     t = spinup_matrix(:,1) - spinup_matrix(1,1);
%     omega = spinup_matrix(:,3);

    t = dt*(0:1:length(hall_state)-1);
    
    figure;
    plot(omega);
    xlabel('sample index');
    ylabel('omega')
    title('raw speed estimates')

    % lowpass filter the raw speed signal (omega):
    df_speed = designfilt('lowpassiir','FilterOrder',3,'HalfPowerFrequency',0.001);
    filt_speed = filtfilt(df_speed,omega);

    % approximate acceleration by differencing the fitted speed:
    accel = [0, diff(filt_speed)/dt];

    figure;
    subplot(2,1,1),plot(t,omega,'k--');ylabel('ang. vel. [deg/s]');hold on;
    plot(t,filt_speed,'k-');
    hold off;legend('raw','fit','Location','Best')
    title('Imported spinup data');
    subplot(2,1,2),plot(t,accel,'k.-');ylabel('ang. acc. [deg/s^2]');
    xlabel('time [s]');

end
