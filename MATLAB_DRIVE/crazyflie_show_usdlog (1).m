% crazyflie load usd card log csv file

% read file
csvfilename = '2024-04-04_log07.csv';

array = dlmread(csvfilename,',',1,0);
%T = table2array(readtable(csvfilename)); % Matlab only

% get data from table (octave)
t = array(:,1)'*1e-3;
dt = mean(t(2:37071)-t(1:37071-1)); % estimate of sample time
pos = array(:,2:4)';
vel = array(:,5:7)';
lbd = array(:,8:10)'*pi/180;
om = array(:,11:13)'*pi/180;
pos_ref = array(:,14:16)';
yaw_ref = array(:,17)';
motors = array(:,18:21)';
%Eixo x
data_x.pos     = pos(1,:);      % stateEstimate.x
data_x.vel     = vel(1,:);      % stateEstimate.vx
data_x.ref     = pos_ref(1,:);  % ctrltarget.x
data_x.omega   = om(1,:);       % gyro.x
data_x.angle   = lbd(2,:);      % pitch
%Eixo y
data_y.pos     = pos(2,:);      % stateEstimate.y
data_y.vel     = vel(2,:);      % stateEstimate.vy
data_y.ref     = pos_ref(2,:);  % ctrltarget.y
data_y.omega   = om(2,:);       % gyro.y
data_y.angle   = lbd(1,:);      % roll
%Eixo z
data_z.pos     = pos(3,:);      % stateEstimate.z
data_z.vel     = vel(3,:);      % stateEstimate.vz
data_z.ref     = pos_ref(3,:);  % ctrltarget.z
data_z.omega   = om(3,:);       % gyro.z
data_z.angle   = lbd(3,:);      % yaw
figure;


subplot(3,1,1)
plot(t, pos(1,:), 'b', t, pos_ref(1,:), 'r--')
ylabel('x [m]')
legend('x','x_{ref}')
title('Position along X axis')

subplot(3,1,2)
plot(t, pos(2,:), 'b', t, pos_ref(2,:), 'r--')
ylabel('y [m]')
legend('y','y_{ref}')
title('Position along Y axis')
xlabel('Time [s]')

subplot(3,1,3)
plot(t, pos(3,:), 'b', t, pos_ref(3,:), 'r--')
ylabel('z [m]')
xlabel('Time [s]')
legend('z','z_{ref}')
title('Position along Z axis')

figure;
subplot(3,1,1)
plot(t, vel(1,:))
ylabel('v_x [m/s]')
title('Velocity along X')
xlabel('Time [s]')
subplot(3,1,2)
plot(t, vel(2,:))
ylabel('v_y [m/s]')
title('Velocity along Y')

subplot(3,1,3)
plot(t, vel(3,:))
ylabel('v_z [m/s]')
xlabel('Time [s]')
title('Velocity along Z')
% convert date to print format
t = time - time(1);
x = [pos;vel;lbd;om];
x_ref = [pos_ref;0*vel;lbd*0;om*0];
x_ref(9,:) = yaw_ref;
uint16_max = 2^16;
u = motors/uint16_max;
a
% plot data
initPlots;
vehicle3d_ref_show_data(t,x,u,x_ref);

% prepare data for ID
% dt = diff(t);
% dt = [dt,dt(end)];
% sample_time_stats = [mean(dt),min(dt),max(dt)],
% Ts = sample_time_stats(1);
u_id = lbd(1,:)';
y_id = pos(2,:)';