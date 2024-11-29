%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inverted pendulum control with LQ control
% Assumes access to the state
%
% J. Miranda Lemos, 2022
% Distributed Estimation and Predictive Control
%
% Edited by: 
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%__________________________________________________________________________
% Defines plant parameters
%
g = 9.8; % (m/s^2), gravity acceleration
L = 0.3; % (m) length of the pendulum
m = 0.3; % (kg), mass of the pendulum
k = 0.01; % (), friction coefficient

a = g / L;
b = k / m;
c = 1 / (m * L ^ 2);

umax = 0.34; % maximum value of the control (N.m) 0.04 for 5º

%__________________________________________________________________________
% Linearized model in continuous time

Ap = [0 1; a -b];
Bp = [0; c];
Cp = [1 0]; % assumes as poutput the angle
Dp = 0;

%__________________________________________________________________________
% Equivalent discrete-time model with ZOH

% Defines the continuous time model
SYSC = ss(Ap, Bp, Cp, Dp);

% Converts it to discrete time

h = 0.1; %(s) sampling period
SYSD = c2d(SYSC, h);

Apd = SYSD.a;
Bpd = SYSD.b;

%__________________________________________________________________________
% LQ controller design (discrete time)clear all

Q = 1;
R = 0.001;

[KLQ,SLQ,ELQ] = dlqr(Apd,Bpd,Q,R);

%__________________________________________________________________________
% Simulates the controlled plant

Tmax = 10; % (s) Duration of the simulation
sim('LinPendLQ', Tmax);

% Check if system converges and the time instant the system converges to 
% theta = 0 at a margin of pi/360 of the final value in range t in [0; 10] s
t_conv = -1;
for k = 1 : size(t,1)
    if t_conv == -1 && -pi/360 <= theta(k) && theta(k) <= pi/360 && k < 0.8 * size(t, 1)
        t_conv = t(k);
    elseif t_conv ~= -1 && (pi/360 < theta(k) || theta(k) < -pi/360)
        t_conv = -1;
    end
end

% Overshoot or undershoot in this case is equal to MIN (|theta_min, theta_max|)
theta_max = max(theta);
theta_min = min(theta);
S = min(abs(theta_max), abs(theta_min));

%__________________________________________________________________________
% Plots results
figure

if t_conv ~= -1
    str2 = sprintf('u_{lim} = %.2f Nm, S = %.3frad, t_{conv π/360} = %.2fs', umax, S, t_conv);
else
    str2 = sprintf('u_{lim} = %.2f Nm, Not Converge', umax);
end
txt = str2;

subplot(211)
gg = plot(t, theta);
set(gg, 'LineWidth', 1.5);
gg = xlabel('Time (s)');
set(gg, 'FontSize', 14);
gg=ylabel('\theta (rad)');
set(gg, 'FontSize', 14);
grid minor
text(1, 1, txt, 'FontSize', 10,'FontWeight','bold', 'Position', [-2.331307836682699,0.433994117731747,0], 'Interpreter', 'tex')

subplot(212)
gg = stairs(u.time, u.signals.values);
set(gg, 'LineWidth', 1.5);
gg = xlabel('Time (s)');
set(gg, 'FontSize', 14);
gg=ylabel('$u$ (Nm)');
set(gg, 'FontSize', 14, 'Interpreter', 'latex');
grid minor

%__________________________________________________________________________
% EoF
