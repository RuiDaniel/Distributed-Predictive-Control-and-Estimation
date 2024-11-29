% Our reference point is x_opt1. 
% Then, starting from it, a wide range of possible directions (different 
% angles) are explored. For each direction, we search for the boundary by 
% varying the radius using the following method. In the initial stage, 
% the radius increases as a function of 2^n - 1, multiplied by the 
% initial step, being n how many points we have advanced in that 
% direction until we reach the maximum step value, 4. This value was 
% experimentally determined by a series of tests. After finding a point 
% outside of the convergence region, the search method changes. 
% From here on, each time a point is found outside (inside), 
% the radius decreases (increases) half of the absolute value of the
% current step and the step is updated. This will continue until the step
% is lower than a certain threshold. With this approach, in order to find 
% the boundary, we only need to store the limit point of the convex 
% convergence region for each direction.
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
clear
close all

% Range of independent variables to consider 
x1min = -80;
x1max = 5;
x2min = -80;
x2max = 5;

% Tolerance to check if point belongs to convergence region
error_tolerance = 10 ^ -4;

% Tolerance to find a point near the chosen minimum
error_tolerance_radius = 0.2;

figure(1), 
axis([x1min x1max x2min x2max]), axis square
hold on

options = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');

boundary_region = zeros(2, 2, 1);
kk = 1;

% Find minimizer to study
x0 = [-2; -2];
selected_min = fminunc(@P1_2_Function, x0, options);
gg = plot(selected_min(1), selected_min(2), 'xk', 'DisplayName', 'x_{opt_1}');
set(gg, 'Linewidth', 1.5);

% Number of angle directions to consider
num_dir = 360 * 1.2;

step_default = 0.5;
step_radius = step_default;
step_max = 4;

% After finding a point outside of the convergence region, the search
% method changes (beggining becomes false)
beginning = true;

polar_num = zeros(2, 1);
aux = zeros(2, 1);
radius = 0;

% Number of optimizations
num_optim = 0;

% We use as reference point a chosen minimizer and then we explore a wide 
% range of possible directions from it by varying the angle. 
% For each direction, if the point in analysis belongs to the convergence region, 
% we move to the next point (gradually increasing the radius), otherwise, we stop
% searching in that direction.
for angle = 0 : 2 * pi() / num_dir : 2 * pi()
    beginning = true;

    % Start with the step radius adequated to where we expect to find the
    % boundary: boundary of angle i ≈ boundary of angle i - 1
    step_radius = radius;
    if step_radius > step_max
        step_radius = step_max;
    elseif step_radius < step_default
        step_radius = step_default;
    end
    radius = 0;
    
    while abs(step_radius) > error_tolerance_radius
        radius = radius + step_radius;
        [polar_num(1), polar_num(2)] = pol2cart(angle, radius);
        x0 = selected_min + polar_num;
    
        % Initial estimate of the minimum
        xopt = fminunc(@P1_2_Function, x0, options);
        num_optim = num_optim + 1;

        % Check if point is in convergence region
        if (sum((xopt - selected_min).^2) < error_tolerance)
            if  beginning == true
                if step_radius * 2 < step_max
                    step_radius = step_radius * 2;
                end
            else
                step_radius = abs((step_radius / 2));
            end
        else
            beginning = false;
            step_radius = - abs((step_radius / 2));
        end
    end

    % Find the boundary for a certain direction
    if (sum((xopt - selected_min).^2) < error_tolerance)
        [polar_num(1), polar_num(2)] = pol2cart(angle, radius);
    else
        [polar_num(1), polar_num(2)] = pol2cart(angle, radius - step_radius);
    end
    boundary_region(kk, :) = selected_min + polar_num;
    kk = kk + 1;
end

num_optim

% Plots boundary of convergence region
gg = plot(boundary_region(:, 1), boundary_region(:, 2), '.-', 'DisplayName', 'Boundary');
set(gg, 'Linewidth', 1.5);

% Identifies axis
gg = xlabel('$x_{1}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);

gg = ylabel('$x_{2}$', Interpreter = 'latex');
set(gg, 'FontSize', 20);

legend(Location = "northoutside", FontSize = 12, NumColumns = 3)

grid minor
hold off
