function [] = animate(T, param,X)

nSteps = length(T);
Xc_all = zeros(param.N, nSteps);  % Dove N è il numero di link
Yc_all = zeros(param.N, nSteps);
for k = 1:nSteps
    x_k = X(k,:)';  % vettore colonna
    [~, x_c, y_c, ~, ~, ~] = dynamicModel(T(k), x_k, param);    
    Xc_all(:,k) = x_c;
    Yc_all(:,k) = y_c;
end
if param.Plot3D == 0
%% Graphic Visualization of the Snake
x_history = [];
x_start = mean(Xc_all(:,1));

y_strip = -0.4;   % Altezza verticale della striscia
h_strip = 0.02;   % Spessore verticale della striscia

%% Creazione Video
name = ['snake'];
vidfile = VideoWriter(name,'Motion JPEG AVI');
open(vidfile);

%Crea figura con dimensioni fisse (560x420) e rimuovi bordi
%% loop through frames
figure('Position',[200 100 1000 600]);
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white')

axis tight manual;  % Impedisce cambiamenti automatici degli assi
hold on;
grid on;
axis equal;

% Regola gli assi per riempire l'intera figura
set(gca, 'Position', [0, 0, 1, 1]);  % [left, bottom, width, height] in normalized units

%figure (se non si vuole creare il video)
for k = 1:length(T)
    clf;
    hold on; grid on;axis square;axis equal;

    % Se attivo il contatto con le pareti, disegna i bordi del tubo
    if param.contact == 1
        % Lunghezza del tubo (visualizzazione): centrata sul baricentro
        x_c = mean(Xc_all(:,k));
        L_view = 2;  % lunghezza visiva del tratto del tubo
        x_min = x_c - L_view;
        x_max = x_c + L_view;

        % Coordinate pareti
        y1 = -param.diameter/2;
        y2 = +param.diameter/2;

        % Disegna le due pareti come linee
        plot([x_min, x_max], [y1, y1], 'k--', 'LineWidth', 2);  % parete inferiore
        plot([x_min, x_max], [y2, y2], 'k--', 'LineWidth', 2);  % parete superiore
    end
    title(sprintf('t = %.2f s', T(k)));
    
    for i = 1:param.N
        xc = Xc_all(i, k);
        yc = Yc_all(i, k);
        theta = linspace(0, 2*pi, 100);
        r = param.l;
        x_circle = xc + r * cos(theta);
        y_circle = yc + r * sin(theta);
        fill(x_circle, y_circle, 'green', 'FaceAlpha', 0.5, 'EdgeColor', 'k');
    end
    
   plot(Xc_all(:,k), Yc_all(:,k), '--', 'Color', [1, 0.5, 0.5], 'LineWidth', 1.5); % spina
   % --- Calcola baricentro del serpente ---
    x_c = mean(Xc_all(:,k));  % baricentro asse X
    x_history(end+1) = x_c;
    x_margin = 1.5;            % quanto "zoom out" vuoi fare
    xlim([x_c - x_margin, x_c + x_margin]);
    ylim([-1, 1]);

    % --- Calcola estremi della scia ---
    x1 = x_start;
    x2 = x_c;

    x_rect = [min(x1,x2), max(x1,x2)];
    width = abs(x2 - x1);

    % --- Disegna la striscia (scia) solo se il serpente si muove ---
    if width > 0.001
        fill([x_rect(1), x_rect(2), x_rect(2), x_rect(1)], ...
             [y_strip, y_strip, y_strip + h_strip, y_strip + h_strip], ...
             [0.1, 0.6, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4);
    end

    % --- Disegna punto attuale del baricentro ---
    plot(x_c, y_strip + h_strip/2, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

    % Cattura frame e scrivi nel video
    writeVideo(vidfile, getframe(gcf));
    drawnow;
end
close(vidfile);


%% Impostazioni per animazione 3D
else
param.dimensionPlot3D = 1;  % Abilita la modalità 3D
nSteps = length(T);
Xc_all = zeros(param.N, nSteps);  % Dove N è il numero di link
Yc_all = zeros(param.N, nSteps);
Zc_all = zeros(param.N, nSteps);  % Aggiunta coordinata Z

for k = 1:nSteps
    x_k = X(k,:)';  % vettore colonna
    [~, x_c,y_c] = dynamicModel(T(k), x_k, param);
    %% Per animazione 3D
    % Calcola la posizione Z (altezza nel tubo)
    z_c = zeros(param.N,1);
    for i = 1:param.N
        % Modello semplificato per altezza nel tubo (puoi personalizzare)
        z_c(i) = 0.1 * sin(0.5 * T(k) + i * 0.2);
    end
    Xc_all(:,k) = x_c;
    Yc_all(:,k) = y_c;
    Zc_all(:,k) = z_c;
end

name = ['snake_3D'];
vidfile = VideoWriter(name, 'Motion JPEG AVI');
open(vidfile);

% Crea figura 3D con dimensioni fisse
fig = figure('Position', [100, 100, 560, 420], ...
             'MenuBar', 'none', ...
             'ToolBar', 'none', ...
             'Resize', 'off');
         
axis tight manual;
hold on;
grid on;
axis equal;
view(3);  % Vista 3D
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Snake Robot in Pipeline - 3D Simulation');

% Disegna il tubo (pipeline)
tube_radius = param.diameter/2;
tube_length = max(Xc_all(:)) - min(Xc_all(:)) + 2;
[x_tube, y_tube, z_tube] = cylinder(tube_radius, 50);
z_tube = z_tube * tube_length;
surf(z_tube, y_tube, x_tube, ...
    'FaceAlpha', 0.2, ...
    'EdgeColor', 'none', ...
    'FaceColor', [0.7 0.7 1]);

% Imposta limiti assi
xlim([min(Xc_all(:))-0.5, max(Xc_all(:))+0.5]);
ylim([-tube_radius-0.1, tube_radius+0.1]);
zlim([-tube_radius-0.1, tube_radius+0.1]);

for k = 1:length(T)
    clf(fig);
    set(fig, 'Color', 'w');
    hold on;
    grid on;
    axis equal;
    view(3);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(sprintf('3D Simulation - t = %.2f s', T(k)));
    
    % Disegna il tubo
    surf(z_tube + min(Xc_all(:)) - 1, y_tube, x_tube, ...
        'FaceAlpha', 0.2, ...
        'EdgeColor', 'none', ...
        'FaceColor', [0.7 0.7 1]);
    
    % Disegna i link del serpente come sfere
    [xs, ys, zs] = sphere(20);
    for i = 1:param.N
        xc = Xc_all(i, k);
        yc = Yc_all(i, k);
        zc = Zc_all(i, k);
        
        % Scala la sfera al raggio del link
        sphere_x = xc + param.l * xs;
        sphere_y = yc + param.l * ys;
        sphere_z = zc + param.l * zs;
        
        surf(sphere_x, sphere_y, sphere_z, ...
            'FaceColor', 'green', ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.8);
    end
    
    % Disegna la spina dorsale in 3D
    plot3(Xc_all(:,k), Yc_all(:,k), Zc_all(:,k), ...
        '--', 'Color', [1, 0.3, 0.3], 'LineWidth', 2);
    
    
    % Imposta limiti assi dinamici
    xlim([min(Xc_all(:))-0.5, max(Xc_all(:))+0.5]);
    ylim([-tube_radius-0.1, tube_radius+0.1]);
    zlim([-tube_radius-0.1, tube_radius+0.1]);
    
    % Cattura frame e scrivi nel video
    frame = getframe(fig);
    writeVideo(vidfile, frame);

    drawnow;
end
close(vidfile);
end
end