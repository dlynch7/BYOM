%% BYOM_Analysis.m
%
% Description:
%   
%
% Author: Dan Lynch
% Date: Jan. 8, 2019

clear;
close all;
clc;

% import data
spinup_matrix = csvread([pwd,'/data/spinup.csv'],2,0); % enter file location here
t = spinup_matrix(:,1) - spinup_matrix(1,1);
omega = spinup_matrix(:,3);

% rough fit as sum of 2 exponentials:
f = fit(t,omega,'exp2');
ff = f.a*exp(f.b*t) + f.c*exp(f.d*t);

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