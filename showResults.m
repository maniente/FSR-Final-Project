function [] = showResults(T, param,X)

nSteps = length(T);
FR = zeros(param.N, nSteps);            % forza propulsiva - senza pareti
FCN = zeros(param.N, nSteps);           % forza normale (contatto parete)
FCT = zeros(param.N, nSteps);           % forza tangenziale (contatto parete)
GROUND = zeros(param.N, nSteps);       % forza propulsiva
Xc_all = zeros(param.N, nSteps);
Yc_all = zeros(param.N, nSteps);

%% Forces Plot
for k = 1:nSteps
    x_k = X(k,:)';  % vettore colonna
    [~, x_c, y_c, fr, fcontact, ground] = dynamicModel(T(k), x_k, param);    
    % Decomposizione delle forze
    FR(:,k) = fr(1,:)';                        % Forze propulsiva - senza pareti
    FCT(:,k) = fcontact(1:param.N);            % Forze tangenziali da parete
    FCN(:,k) = fcontact(param.N+1:end);        % Forze normali da parete
    GROUND(:,k) = ground(1,:)';                % Forze propulsiva - con pareti
    Xc_all(:,k) = x_c;
    Yc_all(:,k) = y_c;
end

% ---- Calcolo della distanza percorsa dal baricentro ----
x_history = mean(Xc_all, 1);  % vettore della posizione del baricentro nel tempo
distance_travelled = abs(x_history(end) - x_history(1));

% ---- Plot ----
figure('Name','Distance');
plot(T, x_c, 'LineWidth', 2.5, 'Color', [0.1 0.6 0.8], 'LineStyle', '-');
hold on;
% Linea orizzontale di riferimento
yline(0, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2);
% Etichette e stile
xlabel('Time [s]', 'FontSize', 14);
ylabel('Distance [m]', 'FontSize', 14);
title(sprintf('Total snake distance: %.3f m', distance_travelled), 'FontSize', 16);
grid on;
set(gca, 'FontSize', 12);


figure('Name', 'Contact Forces', 'NumberTitle', 'off');
% --- Subplot 1: Forza tangenziale
subplot(2, 1, 1);
plot(T, FCT(1,:), 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Tangential Force [N]');
title('Contact Forces - Tangential');
grid on;

% --- Subplot 2: Forza normale
subplot(2, 1, 2);
plot(T, FCN(1,:), 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Normal Force [N]');
title('Contact Force - Normal');
grid on;
%% Forza propulsiva
figure('Name', 'Forze Propulsive', 'NumberTitle', 'off');
if param.contact == 1
    plot(T, GROUND(1,:), 'r', 'LineWidth', 1.5);
else
    plot(T, FR(1,:), 'r', 'LineWidth', 1.5);
end
xlabel('Time [s]');
if param.friction == 1
ylabel('Viscous Friction [N]');
else
ylabel('Coulomb Friction [N]');
end
title('Forza Propulsiva [N]');
grid on;
end