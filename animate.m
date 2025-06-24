function [] = animate(T, param,X)

nSteps = length(T);
Xc_all = zeros(param.N, nSteps);  % Dove N è il numero di link
Yc_all = zeros(param.N, nSteps);
FRX = zeros(param.N,nSteps);       % Forza propulsiva - senza pareti - su x
FRY = zeros(param.N,nSteps);       % Forza propulsiva - senza pareti - su y
FCN = zeros(param.N, nSteps);           % forza normale (contatto parete)
FCT = zeros(param.N, nSteps);           % forza tangenziale (contatto parete)
%XS1 = zeros(nSteps);
%YS1 = zeros(nSteps);
for k = 1:nSteps
    x_k = X(k,:)';  % vettore colonna
    [~, x_c, y_c, fr, fcontact, ~, ~, ~,x_s1,y_s1,p] = dynamicModel(T(k), x_k, param);    
    Xc_all(:,k) = x_c;
    Yc_all(:,k) = y_c;
    FR(:,k) = fr(1,:)';                        % Forze propulsiva - senza pareti
    FRX(:,k) = fr(1:param.N);                  % Forza propulsiva - senza pareti - su x
    FRY(:,k) = fr(param.N+1:end);              % Forza propulsiva - senza pareti - su y
    FCT(:,k) = fcontact(1:param.N);            % Forze tangenziali da parete
    FCN(:,k) = fcontact(param.N+1:end);        % Forze normali da parete
    %XS1(:,k) = x_s1;
    %YS1(:,k) = y_s1;

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
%% Force part
    %scale_force = 0.5;  % fattore di scala per la lunghezza delle frecce
    % Se attivo il contatto con le pareti, disegna i bordi del tubo
    % if param.contact == 0
    %     for i = 1:param.N
    %         fr_x = FRX(i,k);
    %         fr_y = FRY(i,k);
    %     if abs(fr_x) > 1e-4 || (fr_y) > 1e-4
    %         xc = Xc_all(i, k);
    %         yc = Yc_all(i, k);
    % 
    %         %Forza tangenziale: verso x
    %         quiver(xc, yc, 50*fr_x, 0, 0, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
    % 
    %         %Forza normale: verso y
    %         quiver(xc, yc, 0, 50*fr_y, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    %     end
    %     end
    % else
    % 
    %     % Lunghezza del tubo (visualizzazione): centrata sul baricentro
    %     x_c = mean(Xc_all(:,k));
    %     L_view = 2;  % lunghezza visiva del tratto del tubo
    %     x_min = x_c - L_view;
    %     x_max = x_c + L_view;
    % 
    %     % Coordinate pareti
    %     y1 = -param.diameter/2;
    %     y2 = +param.diameter/2;
    % 
    %     % Disegna le due pareti come linee
    %     plot([x_min, x_max], [y1, y1], 'k--', 'LineWidth', 1);  % parete inferiore
    %     plot([x_min, x_max], [y2, y2], 'k--', 'LineWidth', 1);  % parete superiore
    % 
    % 
    %     for i = 1:param.N
    %     fn = FCN(i,k);
    %     ft = FCT(i,k);
    % 
    %     % Solo se c'è una forza non nulla
    %     if abs(fn) > 1e-4 || abs(ft) > 1e-4
    %         xc = Xc_all(i, k);
    %         yc = Yc_all(i, k);
    % 
    %         % Forza tangenziale: verso x
    %         quiver(xc, yc, scale_force*ft, 0, 0, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
    %         % 
    %         % % Forza normale: verso y
    %         quiver(xc, yc, 0, scale_force*fn, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    %     end
    %     end
    % end


    title(sprintf('t = %.2f s', T(k)));
    plot(x_s1,y_s1, 'b--', 'LineWidth',1.5);
    
    


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
Zc_all = zeros(param.N, nSteps);  % Aggiunta coordinata Z

for k = 1:nSteps    
    %% Per animazione 3D
    % Calcola la posizione Z (altezza nel tubo)
    z_c = zeros(param.N,1);
    for i = 1:param.N
        % Modello semplificato per altezza nel tubo (puoi personalizzare)
        z_c(i) = 0.1 * sin(0.5 * T(k) + i * 0.2);
    end
    Zc_all(:,k) = z_c;
end

name = ['snake_3D'];
vidfile = VideoWriter(name, 'Motion JPEG AVI');
open(vidfile);

% Crea figura 3D con dimensioni fisse
figure('Position',[200 100 1000 600]);
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white')

axis tight manual;  % Impedisce cambiamenti automatici degli assi
hold on;
grid on;
axis equal;

view(3);  % Vista 3D
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Snake Robot in Pipeline - 3D Simulation');

if param.contact == 1
    % Green pipeline like your figure
    tube_radius = param.diameter / 2;
    tube_length = max(Xc_all(:)) - min(Xc_all(:)) + 2;
    [x_tube, y_tube, z_tube] = cylinder(tube_radius, 50);
    z_tube = z_tube * tube_length;

    % Rotate cylinder: make it horizontal along X-axis
    surf(z_tube + min(Xc_all(:)) - 1, y_tube, x_tube, ...
        'FaceAlpha', 0.3, ...
        'EdgeColor', 'none', ...
        'FaceColor', [0.2 1 0.2]);  % bright green
end

% Imposta limiti assi
xlim([min(Xc_all(:))-0.5, max(Xc_all(:))+0.5]);
ylim([-tube_radius-0.1, tube_radius+0.1]);
zlim([-tube_radius-0.1, tube_radius+0.1]);

for k = 1:length(T)
    clf(fig);
    set(fig, 'Color', 'w');
    hold on; grid on;axis square;axis equal;

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
    writeVideo(vidfile, getframe(gcf));

    drawnow;
end
close(vidfile);
end
end