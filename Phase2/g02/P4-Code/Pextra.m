% Controlling the pendulum with MPC control, with square wave reference.
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
Hp = 2; % Default: 3
Hu = Hp;

zblk = 1;
ublk = 1;

u_max = 0.38; % Default: 100
u_min = -u_max;

du_max = 0.3; % Default: 100
du_min = -du_max;

theta_max = 10000;
theta_min = -10000;

z_max = theta_max * [1; 1]; 
z_min = theta_min * [1; 1];

Q = 1;  
R = 10 ^ (-0.9); % Default: 0.001

W = []; % State estimator not used
V = []; 

cmode = 0; % Feedback of the state
solver = 'qp_as';

theta_0 = pi / 180; % Initial condition for theta
square_amplitude = pi / 180; % Amplitude of reference to track

flagdist = 0; % Absence of disturbance
dist_amplitude = 0;
t_impulse = 0.5; % Disturbance impulse duration (s)

%% MPC inicialization
md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	    du_max, du_min ,u_max ,u_min ,z_max, ...
	    z_min, Q, R, W, V, h, cmode, 'qp_as');

%% Simulates the controlled system
sim('Pendulum_extra');

% Check if system converges and the time instant the system converges to 
% the reference at a margin of 5% of the final value in range t in [0; 10] s
t_conv = -1;
for k = 1 : size(theta.time, 1)
    if t_conv == -1 && 0.95 * pi / 180 <= theta.signals.values(k) && theta.signals.values(k) <= 1.05 * pi / 180 && k < 0.9 * size(theta.time, 1)
        t_conv = theta.time(k);
    elseif t_conv ~= -1 && (1.05 * pi / 180 < theta.signals.values(k) || theta.signals.values(k) < 0.95 * pi / 180)
        t_conv = -1;
    end
end

% Overshoot or undershoot in this case is equal to MIN (|theta_min, theta_max|)
theta_max = max(theta.signals.values);
S = theta_max;

%% Plots results
figure(1)

str = sprintf('H_{p}=%d, Q=%d, R=%.3f, h=%.1fs, |u|≤%.2fNm, |Δu|≤%.2fNm', Hp, Q, R, h, u_max, du_max);
if t_conv ~= -1
    str2 = sprintf('S = %.3frad, t_{conv 0.05} = %.2f s', S, t_conv - 5);
else
    str2 = 'Not Converge';
end

txt1 = str;
txt2 = str2;

% Plots the output
subplot(4, 1, 1)
gg=plot(theta.time, theta.signals.values, 'b', 'DisplayName', '\theta (rad)');
set(gg,'LineWidth',1.5);
grid minor
%hold off
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$\theta$(rad)');
set(gg,'FontSize', 16, Interpreter = 'latex');
text(1, 1, txt1, 'FontSize', 12,'FontWeight','bold')
text(1, 1, txt2, 'FontSize', 10,'FontWeight','bold')


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

% Plots the variation of the control variable
subplot(4,1,4)
gg=stairs(uout.time, duout);
set(gg,'LineWidth', 1.5);
gg=xlabel('time ($s$)');
set(gg,'FontSize', 16, Interpreter = 'latex');
gg=ylabel('$\Delta u$(Nm)');
set(gg,'FontSize', 14, Interpreter = 'latex');
grid minor


