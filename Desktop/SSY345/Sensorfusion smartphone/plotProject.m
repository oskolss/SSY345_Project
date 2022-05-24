showIP
figure('Color','white','Position',[292   475  1165   422]);
hold on, grid on
plot(meas.t', meas.gyr', 'LineWidth', 1)
xlim([0, max(meas.t)])
title 'Gyroscope', ylabel 'angular velocity [rad/s]', xlabel 'time [s]'
legend({'x','y','z'})

figure('Color','white','Position',[292   475  1165   422]);
hold on; grid on;
plot(meas.t, meas.acc, 'Linewidth',1)
xlim([0,max(meas.t)])
title 'Accelerometer', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x', 'y', 'z'})