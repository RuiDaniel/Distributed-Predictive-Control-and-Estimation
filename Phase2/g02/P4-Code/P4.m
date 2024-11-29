% Controlling the pendulum with MPC control.
% Uses MPCtools 1.0 package, J. Akesson, Univ. Lund, 2006.
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%__________________________________________________________________________

clear
close all
path(path, '../')

tfinal = 10; % Duration of the simulation (s)
h = 0.1; % Sampling period (s)

%% Parameters of the discrete model
g = 9.8; % Gravity acceleration (m / s ^ 2)
L = 0.3; % Length of the pendulum (m)
m = 0.3; % Mass of the pendulum (kg)
k = 0.01; % Friction coefficient ()

a = g / L;
b = k / m;
c = 1 / (m * L ^ 2);

%% Linearized model in continuous time
Ap = [0 1; a -b];
Bp = [0; c];
Cp = [1 0];
Dp = 0;

%% Equivalent discrete-time model with ZOH

% Defines the continuous time model
SYSC = ss(Ap, Bp, Cp, Dp);

% Converts it to discrete time
SYSD = c2d(SYSC, h);

Ad = SYSD.A;
Bd = SYSD.B;
Cyd = SYSD.C;

Czd = [1 0];
Dzd = 0;

Ccd = eye(2);
Dcd = [0; 0];

%% Definition of MPC parameters
Hw = 1;
Hp = 3;
Hu = Hp;

zblk = 1;
ublk = 1;

u_max = 100;
u_min = -u_max;

du_max = 100; 
du_min = -du_max;

theta_max = 10000;
theta_min = -10000;

z_max = theta_max * [1; 1]; 
z_min = theta_min * [1; 1];

Q = 1;  
R = 0.001; 

W = []; % state estimator not used
V = []; % 

cmode = 0; % feedback of the state
solver = 'qp_as';

theta_0 = pi / 8; % Initial condition for theta
ref = 0; % Reference to track

flagdist = 0; % Absence of disturbance
dist_amplitude = 0;
t_impulse = 0.5; % Disturbance impulse duration (s)

%% MPC inicialization
md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	    du_max, du_min ,u_max ,u_min ,z_max, ...
	    z_min, Q, R, W, V, h, cmode, 'qp_as');

%% Simulates the controlled system
sim('Pendulum');

% Check if system converges and the time instant the system converges to 
% theta = 0 at a margin of pi/360 of the final value in range t in [0; 10] s
t_conv = -1;
for k = 1 : size(theta.time, 1)
    if t_conv == -1 && -pi/360 <= theta.signals.values(k) && theta.signals.values(k) <= pi/360 && k < 0.8 * size(theta.time, 1)
        t_conv = theta.time(k);
    elseif t_conv ~= -1 && (pi/360 < theta.signals.values(k) || theta.signals.values(k) < -pi/360)
        t_conv = -1;
    end
end

% Overshoot or undershoot in this case is equal to MIN (|theta_min, theta_max|)
theta_max = max(theta.signals.values);
theta_min = min(theta.signals.values);
S = min(abs(theta_max), abs(theta_min));

%% Plots results
figure(1)

str = sprintf('H_{p}=%d, |u|≤%.2fNm, |Δu|≤%.2fNm', Hp, u_max, du_max);
if t_conv ~= -1
    str2 = sprintf('Q=%d, R=%.3f, h=%.1fs, S = %.3frad, t_{conv π/360} = %.2fs', Q, R, h, S, t_conv);
else
    str2 = sprintf('Q=%d, R=%.3f, h=%.1fs, Not Converge', Q, R, h);
end

txt1 = str;
txt2 = str2;

% Plots the output
subplot(4, 1, 1)
gg=plot(theta.time, theta.signals.values, 'b', 'DisplayName', '\theta (°)');
set(gg,'LineWidth',1.5);
grid minor
hold off
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$\theta$(rad)');
set(gg,'FontSize', 16, Interpreter = 'latex');
% text(1, 1, txt1, 'FontSize', 12,'FontWeight','bold')
% text(1, 1, txt2, 'FontSize', 10,'FontWeight','bold')

% Plots dtheta/dt
subplot(4, 1, 2)
gg=plot(thetadot.time, thetadot.signals.values,'b');
set(gg,'LineWidth',1.5);
grid minor
hold off
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$\dot{\theta}$(rad/s)', 'Interpreter', 'latex');
set(gg,'FontSize', 14);

% Plots the control variable
subplot(4,1,3)
gg=stairs(uout.time, uout.signals.values);
set(gg,'LineWidth', 1.5);
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$u$(Nm)');
set(gg,'FontSize', 14, Interpreter = 'latex');
grid minor
duout = zeros(1, size(uout.signals.values, 1) - 1);
duout(1) = 0;
for kk = 2 : size(uout.signals.values, 1)
    duout(kk) = uout.signals.values(kk) - uout.signals.values(kk - 1);
end
% ylim([-0.34 -0.32])

% Plots the variation of the control variable
subplot(4,1,4)
gg=stairs(uout.time, duout);
set(gg,'LineWidth', 1.5);
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$\Delta u$(Nm)');
set(gg,'FontSize', 14, Interpreter = 'latex');
grid minor