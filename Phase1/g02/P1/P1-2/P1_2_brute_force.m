% Performs an optimization for each grid point in order to test 
% if it belongs to the convergence region and find an estimate of the boundary
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

% Range of independent variables to consider 
x1min = -80;
x1max = 5;
x2min = -80;
x2max = 5;

% Tolerance to check if point belongs to convergence region
error_tolerance = 10 ^ -4;

% Inicialization of the selected minimizer
selected_min = [0; 0];

% Number of intervals in the mesh grid
N1 = 200;
N2 = 200;

xv1 = linspace(x1min, x1max, N1);
xv2 = linspace(x2min, x2max, N2);
[xx1,xx2] = meshgrid(xv1, xv2);

axis([x1min x1max x2min x2max]), axis square
hold on

options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');

convergence_region = zeros(2,2,1);
kk = 1;

% Find minimizer to study
x0 = [-2; -2];
selected_min = fminunc(@P1_2_Function, x0, options);
gg = plot(selected_min(1), selected_min(2), 'xk', 'DisplayName', 'x_{opt_1}');
set(gg, 'Linewidth', 1.5);

for ii = 1 : N1
    for jj = 1 : N2
        % Initial estimation
        x0 = [xx1(ii, jj); xx2(ii, jj)];
        xopt = fminunc(@P1_2_Function, x0, options);

        % Check if point belongs to convergence region
        if (sum((xopt - selected_min).^2) < error_tolerance)
            convergence_region(kk,:) = x0;
            kk = kk + 1;
        end
    end
end

% Plots convergence region
gg = plot(convergence_region(:, 1), convergence_region(:, 2), '.g', 'DisplayName', 'Convergence region');
set(gg, 'Linewidth', 1.5);

% Plots boundary of convergence region
k = boundary(convergence_region(:, 1), convergence_region(:, 2), 0.9);
plot(convergence_region(k, 1), convergence_region(k, 2), '-r', 'DisplayName', 'Boundary')
set(gg, 'Linewidth', 1.5);

% Identifies axis
gg = xlabel('x_1');
set(gg, 'FontSize', 14);

gg = ylabel('x_2');
set(gg, 'FontSize', 14);

legend(Location = "northoutside", FontSize = 12, NumColumns = 3)

grid on
hold off
