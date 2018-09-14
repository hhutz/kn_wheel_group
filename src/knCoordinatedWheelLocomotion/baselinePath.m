pathVars_1
subplot(4,3,1)
plot(x_1,y_1)
grid on
axis equal
xlabel('X (meters)')
ylabel('Y (meters)')
subplot(4,3,2)
hold off
plot(time_1,left_front_drive_1)
grid on
xlabel('Time (Seconds)')
ylabel('LF dist (m)')
subplot(4,3,3)
hold off
plot(time_1,right_front_drive_1)
grid on
xlabel('Time (Seconds)')
ylabel('RF dist (m)')
subplot(4,3,4)
hold off
plot(time_1(1:end-1), diff(left_front_drive_1) * sampleFrequency_1, 'magenta')
grid on
hold on
plot(time_1(1:end-1), diff(right_front_drive_1) * sampleFrequency_1, 'green')
plot(time_1(1:end-1), diff(left_rear_drive_1) * sampleFrequency_1, 'blue')
plot(time_1(1:end-1), diff(right_rear_drive_1) * sampleFrequency_1, 'red')
plot(time_1,speed_1, 'black')
xlabel('time (seconds)')
ylabel('wheel speed (m/s)')
subplot(4,3,5)
hold off
plot(time_1,left_front_steer_1, 'red')
grid on
hold on
plot(time_1,right_front_steer_1, 'green')
plot(time_1,left_rear_steer_1, 'blue')
plot(time_1,right_rear_steer_1, 'magenta')
xlabel('time (seconds)')
ylabel('wheel rotation (deg)')
subplot(4,3,6)
hold off
plot(time_1(1:end-1), diff(left_front_steer_1) * sampleFrequency_1, 'magenta')
grid on
hold on
plot(time_1(1:end-1), diff(right_front_steer_1) * sampleFrequency_1, 'green')
plot(time_1(1:end-1), diff(left_rear_steer_1) * sampleFrequency_1, 'blue')
plot(time_1(1:end-1), diff(right_rear_steer_1) * sampleFrequency_1, 'red')
xlabel('time (seconds)')
ylabel('steer speed (deg/s)')
subplot(4,3,7)
plot(time_1,curvature_1)
grid on
xlabel('time')
ylabel('curvature')
subplot(4,3,8)
plot(time_1(1:end-1), diff(curvature_1) * sampleFrequency_1)
grid on
xlabel('time')
ylabel('dk/dt')
subplot(4,3,9)
plot(time_1,speed_1)
grid on
xlabel('time (s)')
ylabel('speed (m/s)')
subplot(4,3,10)
hold off
plot(time_1(1:end-2), diff(diff(left_front_drive_1)) * sampleFrequency_1 ^2, 'magenta')
grid on
hold on
plot(time_1(1:end-2), diff(diff(right_front_drive_1)) * sampleFrequency_1 ^2, 'green')
plot(time_1(1:end-2), diff(diff(left_rear_drive_1)) * sampleFrequency_1 ^2, 'blue')
plot(time_1(1:end-2), diff(diff(right_rear_drive_1)) * sampleFrequency_1 ^2, 'red')
plot(time_1(1:end-1), diff(speed_1) * sampleFrequency_1, 'black')
xlabel('time (seconds)')
ylabel('wheel accel (m/s^2)')
subplot(4,3,11)
hold off
plot(time_1(1:end-2), diff(diff(left_front_steer_1)) * sampleFrequency_1^2, 'magenta')
grid on
hold on
plot(time_1(1:end-2), diff(diff(right_front_steer_1)) * sampleFrequency_1^2, 'green')
plot(time_1(1:end-2), diff(diff(left_rear_steer_1)) * sampleFrequency_1^2, 'blue')
plot(time_1(1:end-2), diff(diff(right_rear_steer_1)) * sampleFrequency_1^2, 'red')
xlabel('time (seconds)')
ylabel('steer accel (deg/s^2)')
subplot(4,3,12)
hold off
plot(left_front_steer_1(1:end-1), diff(left_front_steer_1) * sampleFrequency_1, 'magenta')
grid on
hold on
plot(right_front_steer_1(1:end-1), diff(right_front_steer_1) * sampleFrequency_1, 'green')
plot(left_rear_steer_1(1:end-1), diff(left_rear_steer_1) * sampleFrequency_1, 'blue')
plot(right_rear_steer_1(1:end-1), diff(right_rear_steer_1) * sampleFrequency_1, 'red')
xlabel('steer position (deg)')
ylabel('steer speed (deg/s)')
orient landscape
print -dpdf path_1
