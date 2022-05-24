%printData

%%2a)
figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas.t', meas.gyr', 'LineWidth', 1);
xlim([0, max(meas.t)]);
ylim([-3*10^-3,3*10^-3]);
title 'Gyroscope', ylabel 'angular velocity [rad/s]', xlabel 'time [s]'
legend({'x','y','z'})

figure('Color','white','Position',[369  172  989  669]);
hold on; grid on; 
plot(meas.t', meas.acc', 'LineWidth', 1);
xlim([0, max(meas.t)]);
ylim([-1 12]);
title 'Accelerometer', ylabel 'acceleration [m/s^2]', xlabel 'time [s]'
legend({'x','y','z'})

figure('Color','white','Position',[369  172  989  669]);
hold on; grid on;
plot(meas.t', meas.mag', 'LineWidth', 2)
xlim([0, max(meas.t)])
% ylim([-1 11]);
title 'Magnetometer', ylabel 'magnetic field [uT]', xlabel 'time [s]'
legend({'x','y','z'})

%% 2b) Histograms, mean and covariances

%First remove the NaNs to not affect the mean and covariances

%Gyro
validDataGyro=~isnan(meas.gyr(1,:));
gyroData=meas.gyr(:,validDataGyro);
muGyro=mean(gyroData);
covGyro=cov(gyroData);

%Accelerometer
validDataAcc=~isnan(meas.acc(1,:));
accData=meas.acc(:,validDataAcc);
muAcc=mean(accData);
covAcc=cov(accData);

%Magnetometer
validDataMag=~isnan(meas.mag(1,:));
magData=meas.mag(:,validDataMag);
muMag=mean(magData);
covMag=cov(magData);




N=100;
level=3;
axname = {'x','y','z'};
figure('Color','white');
hold on; grid on;
for i=1:3
    subplot(3,1,i);hold on; grid on;
    histogram(gyroData(:,i), 50, 'Normalization','pdf');
    xlabel([axname{i},' [rad/s]']);
    %Create the gaussian distribution from the mean and covariance
    [x,y] = normpdfHelp(muGyro(i), covGyro(i,i), 3, 100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
    
end
sgtitle('Histogram gyroscope - angular velocity');

figure('Color','white');
for i=1:3
    subplot(3,1,i);hold on; grid on;
    histogram(accData(:,i), 50, 'Normalization','pdf');
    xlabel([axname{i},' [m/s^2]']);
    %Create the gaussian distribution from the mean and covariance
    [x y]=normpdfHelp(muAcc(i),covAcc(i,i),3,100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
    
end
sgtitle('Histogram Accelerometer-acceleration')

figure('Color','white');
for i=1:3
    subplot(3,1,i);hold on; grid on;
    histogram(magData(:,i), 50, 'Normalization','pdf');
    xlabel([axname{i},' [m/s^2]']);
    %Create the gaussian distribution from the mean and covariance
    [x y]=normpdfHelp(muMag(i),covMag(i,i),3,100);
    plot(x,y, 'LineWidth',2, 'DisplayName', sprintf('gaussian N(x; 0, $P_{N|N})$') );
    
end
sgtitle('Histogram magnetometer - magnetic field')

function [x,y] = normpdfHelp(mu, sigma2, level, N)
    x = linspace(mu-level*sqrt(sigma2), mu+level*sqrt(sigma2), N);
    y = normpdf(x, mu, sqrt(sigma2));
end
