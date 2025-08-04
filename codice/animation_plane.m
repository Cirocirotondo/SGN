%% Plot traiettoria 3D
figure;
plot3(pos_ned(:,2), pos_ned(:,1), -pos_ned(:,3), 'b-', 'LineWidth', 1.5); % x=East, y=North, z=Up -> invertiamo z
xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');
grid on;
axis equal;
title('Traiettoria in 3D (NED)');
hold on;

%% Animazione con aeroplano come terna orientata
% Impostazioni
scale = 150;        % lunghezza delle assi della terna (in metri)
pauseTime = ST;   % tempo tra frame

% Terna iniziale
hX = quiver3(0,0,0,0,0,0,'r','LineWidth',2); % asse x (avanti)
hY = quiver3(0,0,0,0,0,0,'g','LineWidth',2); % asse y (destra)
hZ = quiver3(0,0,0,0,0,0,'b','LineWidth',2); % asse z (basso)

% Loop di animazione
for k = 1:10:length(pos_ned)
    % Posizione attuale
    pos = pos_ned(k,:);

    % Estrai rotazione come matrice di rotazione
    R = rotmat(quat(k), 'frame'); % 3x3 rotation matrix (NED->body)
    R = R'; % 3x3 rotation matrix (body->NED)

    % Calcola le 3 direzioni della terna (moltiplicando per scale)
    x_dir = R(:,1) * scale; % asse x (avanti)
    y_dir = R(:,2) * scale; % asse y (destra)
    z_dir = R(:,3) * scale; % asse z (basso)

    % Aggiorna le terne
    set(hX, 'XData', pos(2), 'YData', pos(1), 'ZData', -pos(3), ...
            'UData', x_dir(2), 'VData', x_dir(1), 'WData', -x_dir(3));
    set(hY, 'XData', pos(2), 'YData', pos(1), 'ZData', -pos(3), ...
            'UData', y_dir(2), 'VData', y_dir(1), 'WData', -y_dir(3));
    set(hZ, 'XData', pos(2), 'YData', pos(1), 'ZData', -pos(3), ...
            'UData', z_dir(2), 'VData', z_dir(1), 'WData', -z_dir(3));

    drawnow;
    pause(pauseTime);
end
