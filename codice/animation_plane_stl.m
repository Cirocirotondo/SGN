%% Carica mesh STL dell'aeroplano
stl = stlread('biplano.stl'); % usa un file con orientamento corretto
f = stl.ConnectivityList;
v = stl.Points;
% plane orientation correction: ruota di 180° attorno X e +90° attorno Z
Rx = [1 0 0; 0 -1 0; 0 0 -1];       % flip su Y e Z (180° attorno X)
Rz = [0 -1 0; +1 0 0; 0 0 1];        % +90° attorno Z

v = (Rz * Rx * v')';                % Applica rotazioni


% Prepara la patch una volta sola
fig = figure('Position', [100 100 1500 900]); % [left, bottom, width, height]
plot3(pos_ned(:,2), pos_ned(:,1), -pos_ned(:,3), 'b-', 'LineWidth', 1.5);
xlabel('East [m]'); ylabel('North [m]'); zlabel('Altitude [m]');
grid on; axis equal; hold on; title('Animazione aeroplano');

airplanePatch = patch('Faces', f, 'Vertices', v, ...
                      'FaceColor', [0.8 0.8 1], 'EdgeColor', 'none');

% Luci e vista
camlight; lighting gouraud;
view(3);

% Crea video writer
% video = VideoWriter('C:\Users\wolfl\OneDrive\Desktop', 'MPEG-4');
% video.FrameRate = 30;
% open(video);

% Centra il modello sull'origine (opzionale)
v = v - mean(v); % se necessario

% Fattore di scala del modello
scale = 2.1;

%% Loop di animazione
for k = 1:15:length(pos_ned)
    pos = pos_ned(k,:)';
    q = quaternion(quat_c_b(k,:));
    R = rotmat(q, 'frame'); % 3x3 rotation matrix (NED->body)
    R = R'; % 3x3 rotation matrix (body->NED)

    % Applica trasformazioni: scala, rotazione, traslazione
    vTransformed = (R * (v' * scale))' + pos';

    % Aggiorna la mesh
    set(airplanePatch, 'Vertices', [vTransformed(:,2), vTransformed(:,1), -vTransformed(:,3)]);

    drawnow;

    % frame = getframe(fig);
    % writeVideo(video, frame);
end

% Chiudi video
% close(video);