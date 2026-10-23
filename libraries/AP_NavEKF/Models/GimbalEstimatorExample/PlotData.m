%% plot gyro bias estimates
figure;
plot(EKFlogs.time,EKFlogs.states(7:9,:)/dtSlow*180/pi);
grid on;
ylabel('Gyro Bias Estimate (deg/sec)');
xlabel('time (sec)');
hold on;
plot([min(time),max(time)],[gyroBias,gyroBias]*180/pi);
hold off;

%% plot velocity
figure;
plot(EKFlogs.time,EKFlogs.states(4:6,:));
grid on;
ylabel('Velocity (m/sec)');
xlabel('time (sec)');

%% calculate and plot tilt correction magnitude
figure;
plot(EKFlogs.time,EKFlogs.tiltCorr);
grid on;
ylabel('Tilt Correction Magnitude (rad)');
xlabel('time (sec)');

%% plot velocity innovations
figure;
subplot(3,1,1);plot(EKFlogs.time,[EKFlogs.velInnov(1,:);sqrt(EKFlogs.velInnovVar(1,:));-sqrt(EKFlogs.velInnovVar(1,:))]');
grid on;
ylabel('VelN Innovations (m)');
subplot(3,1,2);plot(EKFlogs.time,[EKFlogs.velInnov(2,:);sqrt(EKFlogs.velInnovVar(2,:));-sqrt(EKFlogs.velInnovVar(2,:))]');
grid on;
ylabel('VelE Innovations (m)');
subplot(3,1,3);plot(EKFlogs.time,[EKFlogs.velInnov(3,:);sqrt(EKFlogs.velInnovVar(3,:));-sqrt(EKFlogs.velInnovVar(3,:))]');
grid on;
ylabel('VelD Innovations (m)');
xlabel('time (sec)');

%% plot compass innovations
figure;
plot(EKFlogs.time,[EKFlogs.decInnov;sqrt(EKFlogs.decInnovVar);-sqrt(EKFlogs.decInnovVar)]');
grid on;
ylabel('Compass Innovations (rad)');
xlabel('time (sec)');

%% plot Euler angle estimates
figure;
subplot(3,1,1);plot(gimbal.time,gimbal.euler(1,:)*180/pi);
ylabel('Roll (deg)');grid on;
subplot(3,1,2);plot(gimbal.time,gimbal.euler(2,:)*180/pi);
ylabel('Pitch (deg)');grid on;
subplot(3,1,3);plot(gimbal.time,gimbal.euler(3,:)*180/pi);
ylabel('Yaw (deg)');
grid on;
xlabel('time (sec)');

%% plot Euler angle error estimates
figure;
subplot(3,1,1);plot(gimbal.time,gimbal.eulerError(1,:)*180/pi);
ylabel('Roll Error (deg)');grid on;
subplot(3,1,2);plot(gimbal.time,gimbal.eulerError(2,:)*180/pi);
ylabel('Pitch Error (deg)');grid on;
subplot(3,1,3);plot(gimbal.time,gimbal.eulerError(3,:)*180/pi);
ylabel('Yaw Error (deg)');
grid on;
xlabel('time (sec)');