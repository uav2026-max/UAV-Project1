clear; clc; close all;

%% 1. Definição de Parâmetros Físicos
m = 0.029;          % Massa do Crazyflie (kg)
g = 9.81;           % Aceleração da gravidade (m/s^2)

% Coeficiente de arrasto aerodinâmico (c_Dx).
c_Dx = 1.32e-4;      

%% 2. Matriz A para OP.1 (Hover)
% Condições de equilíbrio: v_e = 0, lambda_e = 0, w_e = 0

% Blocos 3x3 não nulos do OP.1
A12_op1 = eye(3);
A23_op1 = [0,  g, 0; 
          -g,  0, 0; 
           0,  0, 0];
A34_op1 = eye(3);

% Construção da Matriz A (12x12) para OP.1
A_OP1 = [zeros(3), A12_op1,  zeros(3), zeros(3);
         zeros(3), zeros(3), A23_op1,  zeros(3);
         zeros(3), zeros(3), zeros(3), A34_op1;
         zeros(3), zeros(3), zeros(3), zeros(3)];

%% 3. Matriz A para OP.2 (Horizontal Flight)
% Escolha de uma velocidade de equilíbrio adequada 
V = 2.0; % m/s

% Cálculo do ângulo de pitch de equilíbrio (theta_e)
sin_theta_e = (c_Dx * V^2) / (m * g);
theta_e = asin(sin_theta_e);

% Blocos 3x3 não nulos do OP.2
A12_op2 = [cos(theta_e), 0,  sin(theta_e);
           0,            1,  0;
          -sin(theta_e), 0,  cos(theta_e)];

A13_op2 = [0, -V*sin(theta_e), 0;
           0,  0,              V*cos(theta_e);
           0, -V*cos(theta_e), 0];

A22_op2 = [-(2*c_Dx*V)/m, 0, 0;
           0,             0, 0;
           0,             0, 0];

A23_op2 = [0,              g*cos(theta_e), 0;
          -g*cos(theta_e), 0,              0;
           0,              g*sin(theta_e), 0];

A24_op2 = [0,  0,  0;
           0,  0,  V;
           0, -V,  0];

A34_op2 = [1,  0,  tan(theta_e);
           0,  1,  0;
           0,  0,  1/cos(theta_e)];

% Construção da Matriz A (12x12) para OP.2
A_OP2 = [zeros(3), A12_op2,  A13_op2,  zeros(3);
         zeros(3), A22_op2,  A23_op2,  A24_op2;
         zeros(3), zeros(3), zeros(3), A34_op2;
         zeros(3), zeros(3), zeros(3), zeros(3)];

%% 4. Cálculo dos Valores Próprios (Eigenvalues)
eig_OP1 = eig(A_OP1);
eig_OP2 = eig(A_OP2);

% Exibir os valores na Command Window
disp('--- Valores Próprios para OP.1 (Hover) ---');
disp(eig_OP1);

disp('--- Valores Próprios para OP.2 (Horizontal Flight, V=2m/s) ---');
disp(eig_OP2);

%% 5. Representação Gráfica (Plot no Plano Complexo)
figure('Name', 'Eigenvalues dos Modelos Linearizados', 'Color', 'w');
hold on; grid on;

% Plot OP.1 (Azul)
p1 = plot(real(eig_OP1), imag(eig_OP1), 'bx', 'MarkerSize', 10, 'LineWidth', 2);

% Plot OP.2 (Vermelho)
p2 = plot(real(eig_OP2), imag(eig_OP2), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);

% Linhas de eixo (Origem)
xline(0, 'k--', 'HandleVisibility', 'off'); 
yline(0, 'k--', 'HandleVisibility', 'off');

% Formatação do gráfico
xlabel('Eixo Real (Re)');
ylabel('Eixo Imaginário (Im)');
title('Eigenvalues Map');
legend([p1, p2], {'OP.1 (Hover)', 'OP.2 (Horizontal Flight)'}, 'Location', 'best');

axis([-1 1 -15 15]); 
hold off;

%% --- Matriz B (Igual para OP1 e OP2) ---
% Valores de Inércia (em kg*m^2)
Jx = 1.329e-5;
Jy = 1.333e-5;
Jz = 2.640e-5;

% Sub-blocos B2 e B4 deduzidos na secção 1.7
B2 = [0, 0, 0, 0;
      0, 0, 0, 0;
      1/m, 0, 0, 0];

B4 = [0, 1/Jx, 0,    0;
      0, 0,    1/Jy, 0;
      0, 0,    0,    1/Jz];

% Construção da Matriz B completa (12x4)
B_OP = [zeros(3,4); 
        B2; 
        zeros(3,4); 
        B4];

%% --- 1.9 Controlabilidade e Observabilidade ---

C = eye(12); % Matriz Identidade 12x12 (porque y = x)
n = 12;      % Número de estados

% Cálculo para o OP.1 (Hover)
rank_C1 = rank(ctrb(A_OP1, B_OP));
rank_O1 = rank(obsv(A_OP1, C));

% Cálculo para o OP.2 (Horizontal Flight)
rank_C2 = rank(ctrb(A_OP2, B_OP));
rank_O2 = rank(obsv(A_OP2, C));

% Exibição dos Resultados na Command Window
disp('--- 1.9 Controlabilidade e Observabilidade ---');
fprintf('OP1 (Hover) -> Rank Controllability: %d | Rank Observability: %d\n', rank_C1, rank_O1);
fprintf('OP2 (Horiz) -> Rank Controllability: %d | Rank Observability: %d\n', rank_C2, rank_O2);
