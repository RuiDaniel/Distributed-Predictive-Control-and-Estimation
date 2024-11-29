% Obtain level lines of the function, its unconstrained
% minimums and 3-D view of the function
%
% Matlab sw required: optimization toolbox
% Functions called
%    fminunc - Matlab function (optimization toolbox) for unconstrained
%        minimization
%    P1_2_Function.m - user defined function that defines the function to be
%        minimized
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Plots function level curves

% Range of independent variables to consider 
x1min = -5;
x1max = 5;
x2min = -5;
x2max = 5;

% Number of intervals in the mesh grid
N1 = 100;
N2 = 100;

xv1 = linspace(x1min, x1max, N1);
xv2 = linspace(x2min, x2max, N2);
[xx1, xx2] = meshgrid(xv1, xv2);

ff = zeros(N1, N2);

% Computes the function at the different points of the mesh grid
for ii = 1 : N1
    for jj = 1 : N2
        x = [xx1(ii, jj); xx2(ii, jj)];
        ff(ii, jj) = P1_2_Function(x);
    end
end

% Plots the level curves using the Matlab function contour
Nlevel = 50;  % Number of level curves in the contour plot
LW = 'linewidth'; FS = 'fontsize'; MS = 'markersize';
figure(1), contour(xv1, xv2, ff, Nlevel, LW, 1.2, 'DisplayName', 'Level lines'), colorbar
axis([x1min x1max x2min x2max]), axis square
hold on

options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');

xopts = zeros(2, 2, 1);

%--------------------------------------------------------------------------
% Compute the minimums using each initial condition for optimization
% equal to a point in the grid
for ii = 1 : N1
    for jj = 1 : N2
        x0 = [xx1(ii, jj); xx2(ii, jj)];
        xopts((ii - 1) * N2 + jj, :) = fminunc(@P1_2_Function, x0, options);
    end
end

xopts = unique(round(xopts, 5, "significant"), 'rows')
gg = plot(xopts(:, 1), xopts(:, 2), 'xr', 'DisplayName', 'Unconstrained local minima');
set(gg, 'Linewidth', 1.5);

set(gca, "FontSize", 14);

% Identifies axis
gg = xlabel('$x_{1}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);

gg = ylabel('$x_{2}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);

legend(Location = "northoutside", FontSize = 12, NumColumns = 3)

grid minor
hold off

% Plots the 3d view of the function
figure(2)
surf(xx1, xx2, ff, EdgeColor = "none");
set(gca, "FontSize", 14);
grid minor

% Identifies axis
gg = xlabel('$x_{1}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);
gg = ylabel('$x_{2}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);
gg = zlabel('$f(x)$', Interpreter = 'latex');
set(gg, 'FontSize', 20);
