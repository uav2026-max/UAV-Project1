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

%% --- Alínea 2.2: Gráficos dos Inputs ---
figure('Name', 'Control Inputs', 'Color', 'w');

% Input Eixo X (Pitch - theta)
subplot(3,1,1);
plot(t, lbd(2,:)*180/pi, 'm', 'LineWidth', 1); % Convertido para graus
ylabel('\theta (Pitch) [deg]');
title('Control Inputs ao longo do tempo');
grid on;

% Input Eixo Y (Roll - phi)
subplot(3,1,2);
plot(t, lbd(1,:)*180/pi, 'g', 'LineWidth', 1); % Convertido para graus
ylabel('\phi (Roll) [deg]');
grid on;

% Input Eixo Z (Thrust)
% Vamos plotar o comando normalizado do motor 1 como representação do Thrust
subplot(3,1,3);
plot(t, u(1,:), 'k', 'LineWidth', 1); 
ylabel('Thrust (Norm)');
xlabel('Time [s]');
grid on;

% convert date to print format
t = t - t(1); 
x = [pos;vel;lbd;om];
x_ref = [pos_ref;0*vel;lbd*0;om*0];
x_ref(9,:) = yaw_ref;
uint16_max = 2^16;
u = motors/uint16_max;

% Trajetória 3D (Alínea 2.1) ---
figure('Name', 'Trajetória 3D do Drone', 'Color', 'w');
% trajetória real (azul) e a referência (vermelho tracejado)
plot3(pos(1,:), pos(2,:), pos(3,:), 'b', 'LineWidth', 1.5);
hold on;
plot3(pos_ref(1,:), pos_ref(2,:), pos_ref(3,:), 'r--', 'LineWidth', 1);

% pontos de Início (Verde) e Fim (Vermelho)
plot3(pos(1,1), pos(2,1), pos(3,1), 'go', 'MarkerFaceColor', 'g'); 
plot3(pos(1,end), pos(2,end), pos(3,end), 'ro', 'MarkerFaceColor', 'r'); 

grid on;
xlabel('p_x [m]');
ylabel('p_y [m]');
zlabel('p_z [m]');
title('Vehicle Trajectory 3D');
legend('Simulado/Real', 'Referência', 'Início', 'Fim', 'Location', 'best');
view(3); % perspetiva em 3D
pbaspect([1 1 2.5]); % Proporção visual [X Y Z]
hold off;
% ----------------------------------------------

% plot data
initPlots;
vehicle3d_ref_show_data(t,x,u,x_ref);

% prepare data for ID
% dt = diff(t);
% dt = [dt,dt(end)];
% sample_time_stats = [mean(dt),min(dt),max(dt)];
% Ts = sample_time_stats(1);

u_id = lbd(1,:)';
y_id = pos(2,:)';


%% --- Preparaçao de Dados para a Alínea 2.3 (System Identification) ---

% 1. Encontrar os índices exatos para cada intervalo de tempo da secção 2.1
idx_z = find(t >= 80 & t <= 125);
idx_x = find(t >= 130 & t <= 155);
idx_y = find(t >= 155 & t <= 185);

% Obter o tempo de amostragem médio (Ts) para usar na Toolbox
Ts = mean(diff(t));

% 2. Isolar os Inputs e Outputs (em formato coluna transposta ' )
% Eixo Z (Input: Thrust/u1 , Output: p_z)
% Nota: assumindo que u(1,:) tem a informação de thrust principal do log
u_z = u(1, idx_z)'; 
y_z = pos(3, idx_z)';

% Eixo X (Input: Pitch/theta , Output: p_x)
u_x = lbd(2, idx_x)'; 
y_x = pos(1, idx_x)';

% Eixo Y (Input: Roll/phi , Output: p_y)
u_y = lbd(1, idx_y)'; 
y_y = pos(2, idx_y)';

disp('Dados recortados com sucesso! Escreve "systemIdentification" na Command Window.');
