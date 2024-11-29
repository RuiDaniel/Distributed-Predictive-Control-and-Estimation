% Illustrates the use of the Matlab solver fminunc and fmincon to find 
% the minimum of the Rosenbrock function.
%
% Matlab sw required: optimization toolbox
% Functions called
%    fminunc - Matlab function (optimization toolbox) for unconstrained
%        minimization
%    fmincon - Matlab function (optimization toolbox) for constrained
%        minimization
%    RosenbrockFunction.m - user defined function that defines the function to be
%        minimized
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Plots function level curves

% Range of independent variables to consider 
x1min = -2;
x1max = 2;
x2min = 0;
x2max = 2;

% Number of intervals in the mesh grid
N1 = 200;
N2 = 200;

xv1 = linspace(x1min, x1max, N1);
xv2 = linspace(x2min, x2max, N2);
[xx1, xx2] = meshgrid(xv1, xv2);

ff = zeros(N1, N2);

% Computes the function at the different points of the mesh grid
for ii = 1 : N1
    for jj = 1 : N2
        x = [xx1(ii, jj); xx2(ii, jj)];
        ff(ii, jj) = RosenbrockFunction(x);
    end
end

% Plots the level curves using the Matlab function contour
Nlevel = 10;  % Number of level curves in the contour plot
LW = 'linewidth'; FS = 'fontsize'; MS = 'markersize';
figure(1), contour(xv1, xv2, ff, Nlevel, LW, 1.2, 'DisplayName', 'Level lines'), colorbar
axis([x1min x1max x2min x2max]), axis square
hold on

%--------------------------------------------------------------------------
% Compute the minimum

% Initial estimate of the minimum
x0 = [-1; 1];

% Define the options to be used with the fminunc solver:
% The quasi-newton algorithm is used because it does not require the
% gradient; the default algorithm requires the gradient
options = optimoptions('fminunc', 'Algorithm', 'quasi-newton');

% Uses the solver fminunc to compute the minimum of the function defined in
% the Matlab function defined in the file RosenbrockFunction.m
xopt = fminunc(@RosenbrockFunction, x0, options);

%--------------------------------------------------------------------------
% Computes the constrained minimum associated to the constraint 
% x(1) ≤ 0.5
% Ax ≤ B
% A = [1 0], B = 0.5

Ac = [1 0];
Bc = 0.5;
xoptconstr = fmincon(@RosenbrockFunction, x0, Ac, Bc);

%--------------------------------------------------------------------------
% Plots the initial point as a red circle
gg = plot(x0(1), x0(2), 'or', 'DisplayName', 'Initial Estimate');
set(gg, 'Linewidth', 1.5);

% Plots the final estimate of the unconstrained minimum as a red cross
gg = plot(xopt(1), xopt(2), 'xr', 'DisplayName', 'Unconstrained minimum');
set(gg, 'Linewidth', 1.5);

% Plots the final estimate of the constrained minimum as a red star
gg = plot(xoptconstr(1), xoptconstr(2), '*r', 'DisplayName', 'Constrained minimum');
set(gg, 'Linewidth', 1.5);

% Plots the constraint boundary
pgon = polyshape([0.5 0.5 x1min-1 x1min-1], [x2max+1 -1 -1 x2max+1]);
gg = plot(pgon, 'DisplayName', 'Constraint region');
gg.EdgeColor = 'black';
gg.LineStyle = '-';
grid minor
set(gca, "FontSize", 14);

% Identifies axis
gg = xlabel('$x_1$');
set(gg, 'FontSize', 20, Interpreter = 'latex');

gg = ylabel('$x_2$');
set(gg, 'FontSize', 20, Interpreter = 'latex');

legend(Location = "northoutside", FontSize = 12, NumColumns = 2)
hold off

%--------------------------------------------------------------------------
% Plots the 3d view of the function

figure(2)
surf(xx1, xx2, ff, EdgeColor = "none");
set(gca, "FontSize", 14);
grid minor

% Identifies axis
gg = xlabel('$x_1$');
set(gg, 'FontSize', 20, Interpreter = 'latex');

gg = ylabel('$x_2$');
set(gg, 'FontSize', 20, Interpreter = 'latex');

gg = zlabel('$f(x)$');
set(gg, 'FontSize', 20, Interpreter = 'latex');
