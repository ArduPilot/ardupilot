%% plot gyro bias estimates
figure;
plot(statesLog(1,:)*0.001,statesLog(9:11,:)/dt*180/pi);
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
plot(statesLog(1,:)*0.001,statesLog(6:8,:));
grid on;
ylabel('EKF velocity Innovations (m/s)');
xlabel('time (sec)');