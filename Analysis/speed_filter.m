%% speed_filter.m
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

    % import data
    [csv_file, csv_path] = uigetfile({'*.csv;'},'Select a CSV file');
    spinup_matrix = csvread([csv_path,csv_file],2,0); % enter file location here
    t = spinup_matrix(:,1) - spinup_matrix(1,1);
    omega = spinup_matrix(:,3);
    
    figure;
    plot(omega);
    xlabel('sample index');
    ylabel('omega')
    title('raw speed estimates')

    % lowpass filter the raw speed signal (omega):
    df = designfilt('lowpassiir','FilterOrder',3,'HalfPowerFrequency',0.15);
    ff = filtfilt(df,omega);

    % approximate acceleration by differencing the fitted speed:
    accel = zeros(length(t),1);
    for i = 2:length(t)
        accel(i) = (ff(i) - ff(i-1))/(t(i)-t(i-1));
    end

    figure;
    subplot(2,1,1),plot(t,omega,'k--');ylabel('ang. vel. [deg/s]');hold on;
    plot(t,ff,'k-');
    hold off;legend('raw','fit','Location','Best')
    title('Imported spinup data');
    subplot(2,1,2),plot(t,accel,'k.-');ylabel('ang. acc. [deg/s^2]');
    xlabel('time [s]');

end
