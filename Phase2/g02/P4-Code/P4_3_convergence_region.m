% Controlling the pendulum with MPC control. Using tuned parameters, get
% the convergence region by varying theta_0.
% Uses MPCtools 1.0 package, J. Akesson, Univ. Lund, 2006.
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%__________________________________________________________________________

clear
close all
path(path, '../')

tfinal = 10; % Duration of the simulation (s)
h = 0.13; % Sampling period (s)

% Parameters of the discrete model
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

u_max = 0.3;
u_min = -u_max;

du_max = 0.2; 
du_min = -du_max;

z_max = 10000 * [1; 1]; 
z_min = -z_max;

Q = 1;  
R = 10 ^ -0.9; 

W = []; % State estimator not used
V = [];  

cmode = 0; % Feedback of the state
solver = 'qp_as';

n_samples = 800;

ref = 0; % Reference to track

theta_init = zeros(n_samples, 1);
t_cv = zeros(n_samples, 1);
S = zeros(n_samples, 1);

flagdist = 0; % Absence of disturbance
dist_amplitude = 0;
t_impulse = 0.5; % Disturbance impulse duration (s)

%% Simulation
kk = 1;
% Initially we tested in range ]-pi, pi] and verify that system converges 
% in t < 10 s, for |theta_0| ≤ 0.32 rad
for theta_0 = -0.4 : 0.8 / n_samples : 0.4 
    if theta_0 ~= 0
        md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	    du_max, du_min ,u_max ,u_min ,z_max, ...
	    z_min, Q, R, W, V, h, cmode, 'qp_as');

        sim('Pendulum');

        % Check if system converges and the time instant the system converges to 
        % theta = 0 at a margin of pi/360 of the final value in range t in [0; 1000] s
        t_conv = -1;
        for k = 1 : size(theta.time,1)
            if t_conv == -1 && -pi/360 <= theta.signals.values(k) && theta.signals.values(k) <= pi/360 && k < 0.8 * size(theta.time, 1)
                t_conv = theta.time(k);
            elseif t_conv ~= -1 && (pi/360 < theta.signals.values(k) || theta.signals.values(k) < -pi/360)
                t_conv = -1;
            end
        end

        t_cv(kk) = t_conv;

        % Overshoot or undershoot in this case is equal to MIN (|theta_min, theta_max|)
        theta_max = max(theta.signals.values);
        theta_min = min(theta.signals.values);
        S(kk) = min(abs(theta_max), abs(theta_min));

        theta_init(kk) = theta_0;
        kk = kk + 1;
    end
end

% Get range where theta_0 leads to convergence
min_display = 1;
max_display = size(theta_init, 1);
for kk = 1 : size(theta_init, 1)
    if (kk == min_display && t_cv(kk) == -1)
        min_display = min_display + 1;
    elseif (kk ~= min_display && t_cv(kk) == -1)
        max_display = kk - 1;
        break
    end
end


%% Plots results

% Plots t_conv in function of theta_0
figure(1)
gg=plot(theta_init(min_display : max_display), t_cv(min_display : max_display), 'b', 'DisplayName', 't_{conv_{π/360}}');
set(gg,'LineWidth',1.5);
datatip(gg, theta_init(min_display), t_cv(min_display), 'location', 'northeast', "FontSize", 12);
datatip(gg, theta_init(max_display), t_cv(max_display), 'location', 'northwest', "FontSize", 12);
grid minor
hold off
gg=xlabel('\theta_{0} (rad)');
set(gg,'FontSize', 24, Interpreter = 'tex');
gg=ylabel('t_{conv_{π/360}} (s)');
set(gg,'FontSize', 24, Interpreter = 'tex');
xlim([theta_init(min_display) theta_init(max_display)])
ylim([0 2.7])

% Plots S in function of theta_0
figure(2)
gg=plot(theta_init(min_display : max_display), S(min_display : max_display), 'b', 'DisplayName', 'S');
set(gg,'LineWidth',1.5);
datatip(gg, theta_init(min_display), S(min_display), 'location', 'northeast', "FontSize", 12);
datatip(gg, theta_init(max_display), S(max_display), 'location', 'northwest', "FontSize", 12);
grid minor
hold off
gg=xlabel('\theta_{0} (rad)');
set(gg,'FontSize', 24, Interpreter = 'tex');
gg=ylabel('S (rad)');
set(gg,'FontSize', 24, Interpreter = 'tex');
xlim([theta_init(min_display) theta_init(max_display)])
ylim([0 0.04])

theta_init(max_display)
