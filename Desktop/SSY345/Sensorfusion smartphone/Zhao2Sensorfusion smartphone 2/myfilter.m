function [xhat, meas] = myfilter(calAcc, calGyr, calMag)
% FILTERTEMPLATE  Filter template
%
% This is a template function for how to collect and filter data
% sent from a smartphone live.  Calibration data for the
% accelerometer, gyroscope and magnetometer assumed available as
% structs with fields m (mean) and R (variance).
%
% The function returns xhat as an array of structs comprising t
% (timestamp), x (state), and P (state covariance) for each
% timestamp, and meas an array of structs comprising t (timestamp),
% acc (accelerometer measurements), gyr (gyroscope measurements),
% mag (magnetometer measurements), and orint (orientation quaternions
% from the phone).  Measurements not availabe are marked with NaNs.
%
% As you implement your own orientation estimate, it will be
% visualized in a simple illustration.  If the orientation estimate
% is checked in the Sensor Fusion app, it will be displayed in a
% separate view.
%
% Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).

  %% Setup necessary infrastructure
  import('com.liu.sensordata.*');  % Used to receive data.
  load dataOfFlat
  %% Filter settings
  t0 = [];  % Initial time (initialize on first data received)
  nx = 4;   % Assuming that you use q as state variable.
  % Add your filter settings here.
  Rw = 1.0e-06 *[0.5643    0.1420   -0.0155;0.1420    0.8936    0.0249;-0.0155    0.0249    0.1979];
  Ra = cov_acc;
  Rm = cov_mag;
  
  g0 = mean_acc;
  m0 = [0; 
        sqrt(mean_mag(1)^2 + mean_mag(2)^2); 
        mean_mag(3)];
  L_hat = norm(m0);
  
  % Current filter state.
  x = [1; 0; 0 ;0];
  P = eye(nx, nx);

  % Saved filter states.
  xhat = struct('t', zeros(1, 0),...
                'x', zeros(nx, 0),...
                'P', zeros(nx, nx, 0));

  meas = struct('t', zeros(1, 0),...
                'acc', zeros(3, 0),...
                'gyr', zeros(3, 0),...
                'mag', zeros(3, 0),...
                'orient', zeros(4, 0));
  try
    %% Create data link
    server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    sentinel = onCleanup(@() server.stop());

    server.start();  % Start data reception.

    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.

    %% Filter loop
    while server.status()  % Repeat while data is available
      % Get the next measurement set, assume all measurements
      % within the next 5 ms are concurrent (suitable for sampling
      % in 100Hz).
      data = server.getNext(5);

      if isnan(data(1))  % No new data received
        continue;        % Skips the rest of the look
      end
      t = data(1)/1000;  % Extract current time

      if isempty(t0)  % Initialize t0
        t0 = t;
      end

      acc = data(1, 2:4)';
%       % for task 07:
%       if ~any(isnan(acc))  % Acc measurements are available.
%         [x, P] = mu_g(x, P, acc, Ra, g0);
%         [x, P] = mu_normalizeQ(x, P);
%       end
      % for task 08:
%      if ~any(isnan(acc))  % Acc measurements are available.
%          alpha = 0.1; %define the boundary of inlier or outlier
%          if abs(9.81 - norm(acc)) <= alpha * 9.81
 %             [x, P] = mu_g(x, P, acc, Ra, g0);  
 %             [x, P] = mu_normalizeQ(x, P);
 %             outlier_acc = 0; %outlier token
 %         else
  %            outlier_acc = 1;
  %        end
  %        ownView.setAccDist(outlier_acc) %given function for displaying outliers or inliers, based on the generated token
   %   end
      
      gyr = data(1, 5:7)';
      % task 05:
      if ~any(isnan(gyr))  % Gyro measurements are available.
          % Time Update
         % [x, P] = tu_qw(x, P, gyr, 1/100, Rw); %Time update function of EKF goes here
         % [x, P] = mu_normalizeQ(x, P);
      %else
        %  randomNoise = eye(4) * 0.1;
        %  [x, P] = tu_q(x, P, randomNoise); %Time update function of EKF goes here, without angular measurements
        %  [x, P] = mu_normalizeQ(x, P);
      end
 

      mag = data(1, 8:10)';
      % for task 10:
     % if ~any(isnan(mag))  % Mag measurements are available.
      %  [x, P] = mu_m(x, P, mag, m0, Rm);
       % [x, P] = mu_normalizeQ(x, P);
      %end
      % for task 11:
      if ~any(isnan(mag))  % Mag measurements are available.
          alpha = 0.002; %define the weight of previous (i.e. k-1) estimate expected magnitude
          L_hat = (1-alpha)*L_hat + alpha*norm(mag);
          if abs(L_hat - norm(mag)) <= 0.1 * L_hat %the treshold is set up to be 10% first to see how it goes
              [x, P] = mu_m(x, P, mag, m0, Rm);  
              [x, P] = mu_normalizeQ(x, P);
              outlier_mag = 0; %outlier token
          else
              outlier_mag = 1;
          end
          ownView.setMagDist(outlier_mag) %given function for displaying outliers or inliers, based on the generated token
      end
      
      orientation = data(1, 18:21)';  % Google's orientation estimate.

      % Visualize result
      if rem(counter, 10) == 0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
          if isempty(googleView)
            subplot(1, 2, 2);
            % Used for visualization.
            googleView = OrientationView('Google filter', gca);
          end
          setOrientation(googleView, orientation);
          title(googleView, 'GOOGLE', 'FontSize', 16);
        end
      end
      counter = counter + 1;

      % Save estimates
      xhat.x(:, end+1) = x;
      xhat.P(:, :, end+1) = P;
      xhat.t(end+1) = t - t0;

      meas.t(end+1) = t - t0;
      meas.acc(:, end+1) = acc;
      meas.gyr(:, end+1) = gyr;
      meas.mag(:, end+1) = mag;
      meas.orient(:, end+1) = orientation;
    end
  catch e
    fprintf(['Unsuccessful connecting to client!\n' ...
      'Make sure to start streaming from the phone *after*'...
             'running this function!']);
  end
end
