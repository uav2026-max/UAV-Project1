function dx = crazyflie_nonlinear_dynamics(~, x, u_rotors, params)
% x = [px py pz u v w phi theta psi p q r]'
% u_rotors = [T1 T2 T3 T4]'

%% States
px    = x(1);
py    = x(2);
pz    = x(3);
ub    = x(4);
vb    = x(5);
wb    = x(6);
phi   = x(7);
theta = x(8);
psi   = x(9);
p     = x(10);
q     = x(11);
r     = x(12);

v_b   = [ub; vb; wb];
omega = [p; q; r];

%% Parameters
m      = params.m;
g      = params.g;
J      = params.J;
l      = params.l;
alpha  = params.alpha;
kQ_kT  = params.kQ_kT;
cDx    = params.cDx;
cDy    = params.cDy;
cDz    = params.cDz;

%% Rotor thrusts
T1 = u_rotors(1);
T2 = u_rotors(2);
T3 = u_rotors(3);
T4 = u_rotors(4);

%% Rotation matrix R_B^I = Rz(psi)*Ry(theta)*Rx(phi)
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

R = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
     spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
     -sth,      cth*sphi,                   cth*cphi];

%% Skew-symmetric matrix
S_omega = [  0  -r   q;
             r   0  -p;
            -q   p   0];

%% 1) Kinematics: position
p_dot = R * v_b;

%% 2) Forces in body frame

% Gravity in body frame
f_g = R' * [0; 0; -m*g];
% Equivalent to:
% f_g = [m*g*sin(theta);
%       -m*g*cos(theta)*sin(phi);
%       -m*g*cos(theta)*cos(phi)];

% Quadratic drag in body frame
f_a = [-cDx*ub*abs(ub);
       -cDy*vb*abs(vb);
       -cDz*wb*abs(wb)];

% Propulsion force in body frame
T_total = T1 + T2 + T3 + T4;
f_p = [0; 0; T_total];

%% 3) Moments from propulsion

n_x = l*sin(alpha) * (-T1 - T2 + T3 + T4);
n_y = l*cos(alpha) * (-T1 + T2 + T3 - T4);
n_z = kQ_kT * (-T1 + T2 - T3 + T4);

n_p = [n_x; n_y; n_z];

%% 4) Translational dynamics
v_dot = -S_omega * v_b + (1/m) * (f_g + f_a + f_p);

%% 5) Euler angle kinematics
% lambda_dot = Q(lambda)*omega
Q = [1,  sphi*tan(theta),  cphi*tan(theta);
     0,  cphi,            -sphi;
     0,  sphi/cth,         cphi/cth];

euler_dot = Q * omega;

%% 6) Rotational dynamics
omega_dot = -J \ (S_omega * J * omega) + J \ n_p;

%% Final state derivative
dx = [p_dot;
      v_dot;
      euler_dot;
      omega_dot];
end
