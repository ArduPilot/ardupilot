%% calculate and plot tilt correction magnitude
figure;
plot(angErrLog(1,:)*0.001,angErrLog(2,:));
grid on;
ylabel('Tilt correction magnitude (deg)');
xlabel('time (sec)');

%% plot gyro bias estimates
figure;
plot(statesLog(1,:)*0.001,statesLog(8:10,:)/dt*180/pi);
grid on;
ylabel('Gyro Bias Estimate (deg/sec)');
xlabel('time (sec)');

%% plot Euler angle estimates
figure;
eulLog(4,:) = eulLog(4,:) + pi;
plot(eulLog(1,:)*0.001,eulLog(2:4,:)*180/pi);
grid on;
ylabel('Euler Angle Estimates (deg)');
xlabel('time (sec)');

%% plot velocity innovations
figure;
plot(statesLog(1,:)*0.001,statesLog(5:7,:));
grid on;
ylabel('EKF velocity Innovations (m/s)');
xlabel('time (sec)');