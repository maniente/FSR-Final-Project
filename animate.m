function [] = animate(T, N,l,contact ,diameter,xc_ref,yc_ref, p,fr,fc)
nSteps = length(T);
xs = linspace(0,20,200);
ys = [zeros(1,50) sin(xs(1:140)*pi/7) zeros(1,10)];
%% Creazione Video
name = ['snake'];
vidfile = VideoWriter(name,'Motion JPEG AVI');
open(vidfile);
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

for k = 1 : 30 : nSteps
    clf;
    hold on; grid on;axis square;axis equal;
    title(sprintf('t = %.2f s', T(k)));
    plot(xs,ys, 'b--', 'LineWidth',1.5);
    
    % --- Visualizza baricentro fornito ---
    plot(p(1,k), p(2,k), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % Baricentro
    plot(p(1,1:k), p(2,1:k), 'r-', 'LineWidth', 1.5);

    for i = 1:N
        xc = xc_ref(i,k);
        yc = yc_ref(i,k);
        theta = linspace(0, 2*pi, 100);
        r = l;
        x_circle = xc + r * cos(theta);
        y_circle = yc + r * sin(theta);
        fill(x_circle, y_circle , [252/255, 255/255, 193/255] , 'EdgeColor', 'k');
    end
    
    plot(xc_ref(:,k), yc_ref(:,k), '--', 'Color', [1, 0.5, 0.5], 'LineWidth', 1.5); % spina
    
    x_c = mean(xc_ref(:,k));
    y_c = mean(yc_ref(:,k)); 
    x_margin = 2;            % quanto "zoom out" vuoi fare
    y_margin = 1.5;
    xlim([x_c - x_margin, x_c + x_margin]);
    ylim([y_c - y_margin, y_c + y_margin]);
    
    %% Force
    %Se attivo il contatto con le pareti, disegna i bordi del tubo
    % if contact == 0
    %     for i = 1:N
    %         fr_x = fr(i,k);
    %         fr_y = fr(N+i,k);
    % 
    %     if abs(fr_x) > 1e-5 || abs(fr_y) > 1e-5
    %         %Forza tangenziale: verso x
    %         xc = xc_ref(i,k);
    %         yc = yc_ref(i,k);
    % 
    %         quiver(xc, yc, fr_x, 0, 0, 'Color', 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    % 
    %         %Forza normale: verso y
    %         quiver(xc, yc, 0, fr_y, 0, 'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    %     end
    %     end
    % 
    % else
    % 
    %     for i = 1:N
    %     ft = fc(i,k);
    %     fn = fc(N+i,k);
    % 
    %     % Solo se c'è una forza non nulla
    %     if abs(fn) > 1e-4 || abs(ft) > 1e-4
    %         xc = xc_ref(i, k);
    %         yc = yc_ref(i, k);
    % 
    %         % Forza tangenziale: verso x
    %         quiver(xc, yc, 0.1*ft, 0, 0, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 1);
    % 
    %         % % Forza normale: verso y
    %         quiver(xc, yc, 0, 0.1*fn, 0, 'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    %     end
    %     end
    % end
    % Coordinate pareti
     y1 = -diameter/2;
     y2 = +diameter/2;

     % Disegna le due pareti come linee
     plot([0, 4], [y1, y1], 'k--', 'LineWidth', 1);  % parete inferiore
     plot([0, 4], [y2, y2], 'k--', 'LineWidth', 1);  % parete superiore

    % Cattura frame e scrivi nel video
    writeVideo(vidfile, getframe(gcf));
    drawnow;
end
close(vidfile);
end
