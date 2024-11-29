% Basics on receding horizon control
%
% IST, MEEC, Distributed Predictive Control and Estimation
% Group 2, 2023: Afonso Alemão, José Antunes, Rui Daniel, Tomás Fonseca
%--------------------------------------------------------------------------

clear
close all

% H in range {1, 2, ..., H_max}
H_max = 30;

% 1st plant
A(1) = 1.2;
B(1) = 1;

% 2nd plant
A(2) = 0.8;
B(2) = 1;

color = ['b' 'g' 'r' 'c' 'm'];
color2 = ['k' "#77AC30" "#A2142F" "#0072BD" "#7E2F8E"];

plant = ['x (t + 1) = 1.2 x(t) + u(t)'; 'x (t + 1) = 0.8 x(t) + u(t)'];

Q = 1;
C = 1; % Q = C^T C
R = [0.01, 0.1, 1, 10, 100];

K_LQ = zeros(length(A), length(R));
S = zeros(length(A), length(R));
lambda = zeros(length(A), length(R));
closed_loop_eigenvalues = zeros(length(A), length(R), H_max);
converge_1percent = zeros(length(A), length(R));

K_RH = zeros(length(A), length(R));
observability = zeros(2);
rank_obs = zeros(2,1);
    
for k = 1 : length(A)
    figure(k)
    hold on

    fprintf('Plant: %s\n', plant(k, :));
    for r = 1 : length(R)
        % Compute LQ state feedback gain K_LQ (2.1)
        [K_LQ(k, r), S(k, r), lambda(k, r)] = dlqr(A(k), B(k), Q, R(r));
        fprintf('R = %.2f -> K_LQ = %f, eigenvalue(A - B K_LQ) = %f\n', R(r), K_LQ(k, r), lambda(k, r));

        % Compute the optimal receding horizon gain for different values of
        % the horizon H
        for H = 1 : H_max
            W = zeros(H, H);
            PI = zeros(H, 1);
            e1 = zeros(1, H);
            e1(1) = 1;

            for h = 1 : H
                PI(h) = C * A(k) ^ h;
            end

            for ii = 1 : H
                for jj = 1 : H
                    if (jj > ii)
                        break;
                    end
                    W(ii, jj) = C * B(k) * A(k) ^ (ii - jj);
                end
            end

            I = eye(H);
            M = transpose(W) * W + R(r) * I;
            K_RH(k, r, H) = e1 * M ^ (-1) * transpose(W) * PI;
            
            if converge_1percent(k, r) == 0 && K_LQ(k, r) * 0.99 <= K_RH(k, r, H)
                converge_1percent(k, r) = H;
            end

            closed_loop_eigenvalues(k, r, H) = eig(A(k) - B(k) * K_RH(k, r, H));
        end

        % Make a plot of the gain K_RH as a function of H and 
        % superimpose it on a line that corresponds to the optimal LQ gain K_LQ
        fx = squeeze(K_RH(k, r, :));
        gg = plot(1 : H_max, fx, strcat('-', color(r)), 'DisplayName', strcat('K_{RH}(R=', num2str(R(r)), ')'));
        set(gg, 'Linewidth', 2);

        if r == 5 && k == 1 
            datatip(gg, converge_1percent(1, 5), fx(converge_1percent(1, 5)), 'location', 'southeast', "FontSize", 8);
        elseif r == 4 && k == 1 
            datatip(gg, converge_1percent(1, 4), fx(converge_1percent(1, 4)), 'location', 'northeast', "FontSize", 8);
        elseif r == 3 && k == 1 
            datatip(gg, converge_1percent(1, 3), fx(converge_1percent(1, 3)), 'location', 'southeast', "FontSize", 8);
        elseif r == 2 && k == 1 
            datatip(gg, converge_1percent(1, 2), fx(converge_1percent(1, 2)), 'location', 'southeast', "FontSize", 8);
        elseif r == 1 && k == 1 
            datatip(gg, converge_1percent(1, 1), fx(converge_1percent(1, 1)), 'location', 'northeast', "FontSize", 8);
        elseif r == 5 && k == 2 
            datatip(gg, converge_1percent(2, 5), fx(converge_1percent(2, 5)), 'location', 'northeast', "FontSize", 8);
        elseif r == 4 && k == 2 
            datatip(gg, converge_1percent(2, 4), fx(converge_1percent(2, 4)), 'location', 'southeast', "FontSize", 8);
        elseif r == 3 && k == 2 
            datatip(gg, converge_1percent(2, 3), fx(converge_1percent(2, 3)), 'location', 'southeast', "FontSize", 8);
        elseif r == 2 && k == 2 
            datatip(gg, converge_1percent(2, 2), fx(converge_1percent(2, 2)), 'location', 'southeast', "FontSize", 8);
        elseif r == 1 && k == 2 
            datatip(gg, converge_1percent(2, 1), fx(converge_1percent(2, 1)), 'location', 'northeast', "FontSize", 8);
        end
        
        gg = plot([-1, H_max + 1], [K_LQ(k, r), K_LQ(k, r)], strcat(':'), 'DisplayName', strcat('K_{LQ}(R=', num2str(R(r)), ')'));
        set(gg, 'Linewidth', 1.5, 'Color', color2(r));
        set(gca, "FontSize", 14);
        
    end
    fprintf('\n');

    gg = xlabel('H');
    set(gg, 'FontSize', 20, Interpreter = 'latex');
    xlim([1 H_max])
    
    gg = ylabel('Gain');
    set(gg, 'FontSize', 20, Interpreter = 'latex');

    legend(Location = "northeast", FontSize = 12, NumColumns = 5)

    ylim([0, 1.6])

    grid minor
    
    % title(append('$', plant(k, :), '$'), 'FontSize', 14, Interpreter = 'latex')

    hold off

    % Make a plot of the absolute value of the closed-loop eigenvalue
    figure(k + length(A))

    hold on
    
    title(append('$', plant(k, :), '$'),'FontSize', 14, Interpreter = 'latex')

    for r = 1 : length(R)
        fx = squeeze(abs(closed_loop_eigenvalues(k, r, :)));
        gg = plot(1 : H_max, fx, '-', 'DisplayName', strcat('R=', num2str(R(r))));
        set(gg, 'Linewidth', 1.5);
        if r == 4 && k == 1 
            datatip(gg, 2, fx(2), 'location', 'southwest', "FontSize", 8);
        elseif r == 5 && k == 1 
            datatip(gg, 7, fx(7), 'location', 'southwest', "FontSize", 8);
            datatip(gg, 8, fx(8), 'location', 'northeast', "FontSize", 8);
        end
    end
    
    % Plots the constraint boundary
    pgon = polyshape([-5 -5 H_max + 5 H_max + 5], [1 -0.1 -0.1 1]); %-0.1 just for aesthetics
    gg = plot(pgon, 'DisplayName', 'Stability region');
    gg.EdgeColor = 'black';
    gg.LineStyle = '--';
    
    set(gca, "FontSize", 14);

    gg = xlabel('H');
    set(gg,'FontSize', 20, Interpreter = 'latex');
    xlim([1, H_max]);
    
    gg = ylabel('$|$Closed-loop eigenvalue$|$');
    set(gg,'FontSize', 20, Interpreter = 'latex');
    ylim([0, 1.2])

    legend(FontSize = 12, NumColumns = 3)
    grid minor

    hold off

    observability(k) = obsv(A(k), C);
    rank_obs(k) = rank(observability(k));
end



