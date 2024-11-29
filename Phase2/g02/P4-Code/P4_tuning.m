% Controlling the pendulum with MPC control. Tuning of the parameters
% based on a merit figure.
% Uses MPCtools 1.0 package, J. Akesson, Univ. Lund, 2006.
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%__________________________________________________________________________

clear
close all
path(path, '../')

tfinal = 10; % Duration of the simulation (s)

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
Cp = [1 0]; % assumes as poutput the angle
Dp = 0;

%% Equivalent discrete-time model with ZOH

Czd = [1 0];
Dzd = 0;

% Defines the continuous time model
SYSC = ss(Ap, Bp, Cp, Dp);

Ccd = eye(2);
Dcd = [0; 0];
Hw = 1;
zblk = 1;
ublk = 1;
Q = 1;  % Default: 1
W = []; % State estimator not used
V = []; 

% Default for thetalim: 10000
z_max = 10000 * [1; 1];
z_min = -z_max;

cmode = 0; % Feedback of the state
solver = 'qp_as';

theta_0 = pi / 8; % Initial condition for theta
ref = 0; % Reference to track

flagdist = 0; % Absence of disturbance
dist_amplitude = 2.5;
t_impulse = 0.5; % Disturbance impulse duration (s)

%% Default parameters for tuning:
x = zeros(5, 2); % Pre-allocating
y = zeros(5, 2);

% Default for h: 0.1
h = 0.1; % 
SYSD = c2d(SYSC, h);
Ad = SYSD.A;
Bd = SYSD.B;
Cyd = SYSD.C;

% Default for Hp: 3
Hp = 3;   
Hu = Hp;

% Default for ulim: 100
u_max = 100;
u_min = -u_max;

% Default for dulim: 100
du_max = 100; 
du_min = -du_max;

% Default for Q/R: Q = 1, R = 0.001 => Q/R = 10000
R = 0.001;  

%% Tuning of Hp
for Hp = 2 : 27
    x(1, Hp - 1) = Hp;    
    Hu = Hp;

    md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	        du_max, du_min ,u_max ,u_min ,z_max, ...
	        z_min, Q, R, W, V, h, cmode, 'qp_as');

    sim('Pendulum');

    % Check if system converges and the time instant the system converges to 
    % theta = 0 at a margin of pi/360 of the final value
    t_conv = -1;
    for k = 1 : size(theta.time,1)
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

    if t_conv ~= -1
        merit_figure = R * h / (t_conv * Q * Hp * (S+0.1) * u_max * du_max);
    else
        merit_figure = -1; % Not exist
    end
    y(1, Hp - 1) = merit_figure;
end
Hp = 2; % Tuned value

%% Tuning of Q/R
for kk = -4 : 0.1 : 2.7
    R = 10 ^ kk;
    x(2, round((kk + 4.1) * 10)) = 1 / R;   

    md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	        du_max, du_min ,u_max ,u_min ,z_max, ...
	        z_min, Q, R, W, V, h, cmode, 'qp_as');

    sim('Pendulum');

    % Check if system converges and the time instant the system converges to 
    % theta = 0 at a margin of pi/360 of the final value
    t_conv = -1;
    for k = 1 : size(theta.time,1)
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

    if t_conv ~= -1
        merit_figure = R * h / (t_conv * Q * Hp * (S+0.1) * u_max * du_max);
    else
        merit_figure = -1; % Not exist
    end
    y(2, round((kk + 4.1) * 10)) = merit_figure;
end
R = 10 ^ (-0.9); % Tuned value

%% Tuning of ulim
for u_max = 0.01 : 0.01 : 1
    u_min = -u_max;
    x(3, round(u_max * 100)) = u_max;   

    md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	        du_max, du_min ,u_max ,u_min ,z_max, ...
	        z_min, Q, R, W, V, h, cmode, 'qp_as');

    sim('Pendulum');

    % Check if system converges and the time instant the system converges to 
    % theta = 0 at a margin of pi/360 of the final value
    t_conv = -1;
    for k = 1 : size(theta.time,1)
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

    if t_conv ~= -1
        merit_figure = R * h / (t_conv * Q * Hp * (S+0.1) * u_max * du_max);
    else
        merit_figure = -1; % Not exist
    end
    y(3, round(u_max * 100)) = merit_figure;
end
u_max = 0.38; % Tuned value
u_min = -u_max;

%% Tuning of dulim
for du_max = 0.01 : 0.01 : 0.5
    du_min = -du_max;
    x(4, round(du_max * 100)) = du_max;   

    md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	        du_max, du_min ,u_max ,u_min ,z_max, ...
	        z_min, Q, R, W, V, h, cmode, 'qp_as');

    sim('Pendulum');

    % Check if system converges and the time instant the system converges to 
    % theta = 0 at a margin of pi/360 of the final value
    t_conv = -1;
    for k = 1 : size(theta.time,1)
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

    if t_conv ~= -1
        merit_figure = R * h / (t_conv * Q * Hp * (S+0.1) * u_max * du_max);
    else
        merit_figure = -1; % Not exist
    end
    y(4, round(du_max * 100)) = merit_figure;
end
du_max = 0.3;  % Tuned value
du_min = -du_max;

%% Tuning of h
for h = 0.01 : 0.01 : 0.2
    SYSD = c2d(SYSC, h);
    Ad = SYSD.A;
    Bd = SYSD.B;
    Cyd = SYSD.C;
    x(5, round(h * 100)) = h;   

    md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	        du_max, du_min ,u_max ,u_min ,z_max, ...
	        z_min, Q, R, W, V, h, cmode, 'qp_as');

    sim('Pendulum');

    % Check if system converges and the time instant the system converges to 
    % theta = 0 at a margin of pi/360 of the final value
    t_conv = -1;
    for k = 1 : size(theta.time,1)
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

    if t_conv ~= -1
        merit_figure = R * h / (t_conv * Q * Hp * (S+0.1) * u_max * du_max);
    else
        merit_figure = -1; % Not exist
    end
    y(5, round(h * 100)) = merit_figure;
end
h = 0.13; % Tuned value

%% Plots results: 5 plots of merit figure in fuction of each parameter
for pp = 1 : 5
    min_display = 1;
    max_display = size(y, 2);
    for kk = 1 : size(y, 2)
        if (kk == min_display && y(pp, kk) == -1)
            min_display = min_display + 1;
        elseif (kk ~= min_display && (y(pp, kk) == -1 || y(pp, kk) == 0))
            max_display = kk - 1;
            break
        end
    end

    figure(pp)
    if pp ~= 2
        gg=plot(x(pp, min_display : max_display), y(pp, min_display : max_display), 'b');
    else
        gg=semilogx(x(pp, min_display : max_display), y(pp, min_display : max_display), 'b');
    end
    set(gg,'LineWidth',1.5);
    if x(pp, min_display) < x(pp, max_display)
        xlim([x(pp, min_display) x(pp, max_display)])
    else
        xlim([x(pp, max_display) x(pp, min_display)])
    end
      
    if pp == 1 
        datatip(gg, x(pp, 1), y(pp, 1), 'location', 'northeast', "FontSize", 12);
    elseif pp == 2
        datatip(gg, x(pp, 32), y(pp, 32), 'location', 'northeast', "FontSize", 12);
    elseif pp == 3 
        datatip(gg, x(pp, 38), y(pp, 38), 'location', 'northeast', "FontSize", 12);
    elseif pp == 4 
        datatip(gg, x(pp, 30), y(pp, 30), 'location', 'northeast', "FontSize", 12);
    elseif pp == 5 
        datatip(gg, x(pp, 13), y(pp, 13), 'location', 'northwest', "FontSize", 12);
    end
    grid minor
    if pp == 1 
        gg = xlabel('$H_p$');
    elseif pp == 2
        gg = xlabel('$Q/R$');
    elseif pp == 3 
        gg = xlabel('$u_{lim}$ (Nm)');
    elseif pp == 4 
        gg = xlabel('$\Delta u_{lim}$ (Nm)');
    elseif pp == 5 
        gg = xlabel('$h (s)$');
    end
    set(gg,'FontSize', 24, Interpreter = 'latex');
    gg=ylabel('Merit Figure');
    set(gg,'FontSize', 24, Interpreter = 'latex');
    if pp == 1 
        str = 'Tuning of $H_p$';
        title(str, 'FontSize', 20, Interpreter = 'latex')
        ylim_min = 0;
        ylim_max = 8 * 10 ^ -8;
        ylim([ylim_min ylim_max])
    elseif pp == 2
        str = 'Tuning of $Q/R$ ($H_p = 2$)';
        title(str, 'FontSize', 20, Interpreter = 'latex')
        ylim_min = 0 * 10 ^ (-6);
        ylim_max = 5 * 10 ^ (-6);
        ylim([ylim_min ylim_max])
    elseif pp == 3 
        str = 'Tuning of $u_{lim}$';
        str2 = '($H_p = 2$, $R = 10^{-0.9}$)';
        ylim_min = 4 * 10 ^ (-4);
        ylim_max = 11.5 * 10 ^ (-4);
        ylim([ylim_min ylim_max])
        title({str, str2}, 'FontSize', 20, Interpreter = 'latex')
    elseif pp == 4 
        str = 'Tuning of $\Delta u_{lim}$ ($H_p = 2$,';
        str2 = '$R=10^{-0.9}$, $u_{lim} = 0.38$ Nm)';
        ylim([0 0.35])
        title({str, str2}, 'FontSize', 20, Interpreter = 'latex')
    elseif pp == 5 
        str = 'Tuning of $h$ ($H_p = 2$, $R = 10^{-0.9}$,';
        str2 = '$u_{lim} = 0.38$ Nm, $\Delta u_{lim} = 0.3$ Nm)';
        title({str, str2}, 'FontSize', 20, Interpreter = 'latex')
        ylim([0 0.4])
    end
end
