clc;
clear;
close all;

%% 
% x pitch
% y roll
% z yaw
run('startup.m')
showIP
%%
[xhat, meas] = filterTemplate();
%% 2.a
storedData.xhat = xhat;
storedData.meas = meas;
save('storedData.mat', '-struct', 'storedData')

%% plot the measurements
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas.t', meas.gyr', 'LineWidth', 2);
xlim([0, max(meas.t)]);
ylim([-3*10^-3,3*10^-3]);
title 'Gyroscope', ylabel 'angular velocity [rad/s]', xlabel 'time [s]'
legend({'x','y','z'})
print('task2_2.eps','-depsc');

figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas.t', meas.acc', 'LineWidth', 2);
xlim([0, max(meas.t)]);
ylim([-1 12]);
title 'Accelerometer', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x','y','z'})
print('task2_1.eps','-depsc');

figure('Color','white','Position',[369  172  989  669]);
hold on; grid on;
plot(meas.t', meas.mag', 'LineWidth', 2)
xlim([0, max(meas.t)])
% ylim([-1 11]);
title 'Magnetometer', ylabel 'magnetic field [uT]', xlabel 'time [s]'
legend({'x','y','z'})
print('task2_3.eps','-depsc');

%%
% Histograms of measurements for different sensors and axes
% Accelerometer:
mean_acc = mean(meas.acc(:, ~any(isnan(meas.acc), 1)), 2)
cov_acc = cov(meas.acc(:, ~any(isnan(meas.acc), 1)).')
figure ()
subplot(1,3,1)
histogram(meas.acc(2, ~any(isnan(meas.acc))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('x of Acc')
xlabel('Y_{acc,x} [m/s^2]')
subplot(1,3,2)
histogram(meas.acc(1, ~any(isnan(meas.acc))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('y of Acc')
xlabel('Y_{acc,y} [m/s^2]')
subplot(1,3,3)
histogram(meas.acc(3, ~any(isnan(meas.acc))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('z of Acc')
xlabel('Y_{acc,z} [m/s^2]')
print('task2_4.eps','-depsc');

% Gyroscope:
mean_gyr = mean(meas.gyr(:, ~any(isnan(meas.gyr), 1)), 2)
cov_gyr = cov(meas.gyr(:, ~any(isnan(meas.gyr), 1)).')
figure ()
subplot(1,3,1)
histogram(meas.gyr(2, ~any(isnan(meas.gyr))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('x of gyr')
xlabel('Y_{gyr,x} [rad/s]')
subplot(1,3,2)
histogram(meas.gyr(1, ~any(isnan(meas.gyr))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('y of gyr')
xlabel('Y_{gyr,y} [rad/s]')
subplot(1,3,3)
histogram(meas.gyr(3, ~any(isnan(meas.gyr))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('z of gyr')
xlabel('Y_{gyr,z} [rad/s]')
print('task2_5.eps','-depsc');

% Magnetometer:
mean_mag = mean(meas.mag(:, ~any(isnan(meas.mag), 1)), 2)
cov_mag = cov(meas.mag(:, ~any(isnan(meas.mag), 1)).')
figure ()
subplot(1,3,1)
histogram(meas.mag(2, ~any(isnan(meas.mag))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('x of Mag')
xlabel('Y_{mag,x} [uT]')
subplot(1,3,2)
histogram(meas.mag(1, ~any(isnan(meas.mag))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('y of mag')
xlabel('Y_{Mag,y} [uT]')
subplot(1,3,3)
histogram(meas.mag(3, ~any(isnan(meas.mag))), 'Normalization','pdf', 'FaceColor', '#0095BD')
grid on;
title('z of Mag')
xlabel('Y_{mag,z} [uT]')
print('task2_6.eps','-depsc');

save dataOfFlat.mat
%% task 05:
[xhat_5_1, meas_5_1] = myfilter(); % data when the phone is standing on its side

%% task 07:
[xhat_7_1, meas_7_1] = myfilter();
%%
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas_7_1.t', meas_7_1.acc', 'LineWidth', 2);
xlim([0, max(meas_7_1.t)]);
title 'Accelerometer from Task 07', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x','y','z'})
print('task7.eps','-depsc');
%% task 08:
[xhat_8_1, meas_8_1] = myfilter();
%%
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas_8_1.t', meas_8_1.acc', 'LineWidth', 2);
xlim([0, max(meas_8_1.t)]);
title 'Accelerometer from Task 08', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x','y','z'})
print('task8.eps','-depsc');

%% task 10:
[xhat_10_1, meas_10_1] = myfilter();
%%
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas_10_1.t', meas_10_1.mag', 'LineWidth', 2);
xlim([0, max(meas_10_1.t)]);
title 'Magnetometer from Task 10', ylabel 'magnetic field [uT]', xlabel 'time [s]'
legend({'x','y','z'})
print('task10.eps','-depsc');

%% task 11:
[xhat_11_1, meas_11_1] = myfilter();
%%
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas_11_1.t', meas_11_1.mag', 'LineWidth', 2);
xlim([0, max(meas_11_1.t)]);
title 'Magnetometer from Task 11', ylabel 'magnetic field [uT]', xlabel 'time [s]'
legend({'x','y','z'})
print('task11.eps','-depsc');