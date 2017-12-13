% This file is used to analysis the lateral lqr controller performance, data file 'debug_pp\*'

clc;
close all;
clear;

LineWidth = 2;
plotFlag = 0;
TS = 0.1;
path = 'debug_pp/';

dirOutput = dir(fullfile(path,'*_pp_debug.txt'));
debug_data = load([path,dirOutput(end-1).name]);


time_vec = debug_data(:,1);

time_v = time_vec - time_vec(1);
time_v = (1:length(time_v))*TS;

car_x = debug_data(:,1);
car_y = debug_data(:,2);
lat_err = debug_data(:,3);
lat_err_inter = debug_data(:,4);
lat_err_derivate = debug_data(:,5);
steer_angle = debug_data(:,6);
steer_cur = debug_data(:,7);

h = figurename('position');
plot(car_x,car_y,'-b','LineWidth',LineWidth);
hold on;
grid on;
xlabel('X/[m]'); ylabel('Y/[m]');
legend('position');


h = figurename('lat_info');
subplot 311;
plot(time_v,lat_err,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('lat-err/[m]');
legend('lat-err');
grid on

subplot 312;
plot(time_v,lat_err_inter,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('lat-err-inter/[m]');
legend('lat-err-inter');
grid on

subplot 313;
plot(time_v,lat_err_derivate,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('lat-err-derivate/[m]');
legend('lat-err-derivate');
grid on




h = figurename('steer_info');
plot(time_v,steer_angle,'-r','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('steer/[m]');
hold on
plot(time_v,steer_cur,'-b','LineWidth',LineWidth);
legend('steer-cmd','steer-cur');
grid on

autoArrangeFigures;
return;

