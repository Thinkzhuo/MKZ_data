% This file is used to analysis the lateral lqr controller performance, data file 'debug_pp\*'

clc;
close all;
clear;

LineWidth = 2;
plotFlag = 0;
TS = 0.1;
path = 'debug_lonctrl_pid/';

dirOutput = dir(fullfile(path,'*_lonctrl_pid_debug.txt'));
debug_data = load([path,dirOutput(end).name]);



time_vec = debug_data(:,1);

time_v = time_vec - time_vec(1);
time_v = (1:length(time_v))*TS;

v_des = debug_data(:,1);
v_car = debug_data(:,2);
e_v = debug_data(:,3);
u_v_p = debug_data(:,4);
u_v_i = debug_data(:,5);
u_v_d = debug_data(:,6);

h = figurename('velocity-info');
plot(time_v,v_des,'-r','LineWidth',LineWidth);
hold on;
grid on;
plot(time_v,v_car,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('velocity/[m\cdots^{-1}]');
legend('v-des','v-car');


h = figurename('velocity-error');
plot(time_v,e_v,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('velocity-error/[m\cdots^{-1}]');
legend('velocity-error');
grid on


h = figurename('PID-info');
subplot 311;
plot(time_v,u_v_p,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('u-v-p');
legend('u-v-p');
grid on

subplot 312;
plot(time_v,u_v_i,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('u-v-i');
legend('u-v-i');
grid on

subplot 313;
plot(time_v,u_v_d,'-b','LineWidth',LineWidth);
xlabel('t/[s]'); ylabel('u-v-d');
legend('u-v-d');
grid on

autoArrangeFigures;
return;