% Simulation of a first order open-loop unstable plant controlled by
% an MPC controller using Simulink.
%
% Uses MPCtools 1.0 package, J. Akesson, Univ. Lund, 2006.
%
% Plant to control:
% x(k + 1) = 1.2x(k) + u(k), y(k) = x(k)
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%__________________________________________________________________________

clear
close all
path(path, '../')

tfinal = 150; % duration of the simulation (s)

% Parameters of the discrete model

h = 0.1; % sampling period (s)

Ap = 1.2;
Bp = 1;
Cp = 1;   
Dp = 0;

% Model used by MPC

Ad = Ap;
Bd = Bp;
Cyd = Cp;

Czd = Cyd;
Dzd = Dp;   

Ccd = Cp;
Dcd = Dp;

% Definition of MPC parameters

Hw = 1;
Hp = 3;   % Minimum Hp = 2; corresponds to 1-step prediction horizon; Default: 3
Hu = Hp;

zblk = 1;
ublk = 1;

u_max = 100; % Default: 100
u_min = -u_max;

du_max = 100; % Default: 100
du_min = -du_max;

z_max = 10000; % Default: 10000;
z_min = -z_max;

Q = 1;  % Default: 1
R = 1;  % Default: 1

W = []; % state estimator not used
V = []; % 

cmode = 0; % feedback of the state
solver = 'qp_as';

%--------------------------------------------------------------------------
% MPC inicialization

md = MPCInit(Ad, Bd, Cyd, Czd, Dzd, Ccd, Dcd, Hp, Hw, zblk, Hu, ublk, ...
	    du_max, du_min ,u_max ,u_min ,z_max, ...
	    z_min, Q, R, W, V, h, cmode, 'qp_as');

%--------------------------------------------------------------------------
% Simulates the controlled system
sim('Oscillating');

y_out_max = max(yout);

% Check if system converges and the time instant the system converges to 
% rout = 1 at a margin of 5% of the final value
t_conv_5prc = -1;

for k = 1 : 620
    if t_conv_5prc == -1 && -0.95 >= yout(k) && yout(k) >= -1.05 && k < 0.8*620
        t_conv_5prc = k * h;
    elseif t_conv_5prc ~= -1 && (-0.95 < yout(k) || yout(k) < -1.05)
        t_conv_5prc = -1;
    end
end

if t_conv_5prc ~= -1
    t_conv_5prc_2 = -1;
    for k = 621 : 1250
        if t_conv_5prc_2 == -1 && 0.95 <= yout(k) && yout(k) <= 1.05 && k < 621 + (1250-621)*0.8
            t_conv_5prc_2 = k * h;
        elseif t_conv_5prc_2 ~= -1 && (0.95 > yout(k) || yout(k) > 1.05)
            t_conv_5prc_2 = -1;
        end
    end
    if t_conv_5prc_2 == -1
        t_conv_5prc = -1;
    end
end

%--------------------------------------------------------------------------
% Plots results

figure(1)

% Plots the output and the reference
subplot(2,1,1)
gg=plot(kt,yout,'b', 'DisplayName', 'y_{out}');
set(gg,'LineWidth',1.5);
hold on
gg=plot(kt,rout,'r', 'DisplayName', 'r_{out}');
set(gg,'LineWidth',1.5);
legend(Location = "northwest", FontSize = 12, NumColumns = 1)
grid minor
hold off

gg=xlabel('time ($s$)');
set(gg,'FontSize', 24, Interpreter = 'latex');
gg=ylabel('$y$');
set(gg,'FontSize', 24, Interpreter = 'latex');
str = sprintf('H_{p} = %d, Q = %d, R = %d, |u| ≤ %.2f, |Δu| ≤ %.2f, |z| ≤ %d', Hp, Q, R, u_max, du_max, z_max(1));

if t_conv_5prc ~= -1
    str2 = sprintf('y_{out_{max}} = %.2f, t_{conv_{0.05}} = %.2f s', y_out_max, t_conv_5prc_2);
else
    str2 = sprintf('y_{out_{max}} = %.2f, Not Converge', y_out_max);
end
title({str, str2}, 'FontSize', 18, Interpreter = 'tex')

% Plots the control variable
subplot(2,1,2)

gg=stairs(kt,uout);
set(gg,'LineWidth',1.5);
gg=xlabel('time ($s$)');
set(gg,'FontSize', 24, Interpreter = 'latex');
gg=ylabel('$u$');
set(gg,'FontSize', 24, Interpreter = 'latex');

grid minor
