g = 9.81;  % [m/s^2]

%% options
AutoOrientation = 1;        % 0 to specify orientation in waypoints, 1 for auto-Orientation
use_filtered_acc = 1;       % using true (0) or filtered acceleration (1)

%% trajectory generation

% Generate trajectory in NED system using waypoints
if AutoOrientation == 1
    trajectory = waypointTrajectory(waypoints_ned(:,2:4), ...
    'TimeOfArrival',waypoints_ned(:,1), 'AutoPitch', true, 'AutoBank', true,'SampleRate',1/ST);
else
    trajectory = waypointTrajectory(waypoints_ned(:,2:4), ...
    TimeOfArrival = waypoints_ned(:,1), ...
    Orientation = quaternion(waypoints_ned(:,5:7),"euler","ZYX","frame"), ...
    SampleRate = 1/ST);
end

% waypointInfo returns a table of specified constraints.
tInfo = waypointInfo(trajectory);
t_end = tInfo.TimeOfArrival(end);

%% trajectory samples extraction
trajectory.reset();
time = 0:ST:(tInfo.TimeOfArrival(end)-ST);                                      % time vector
time=time'; 
quat_b = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,1,"quaternion");    % orientation quaternion, from nav to body, expressed in body frame
vel_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);                % velocity in navigation frame [m/s, m/s, m/s]
acc_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);                % acceleration in navigation frame [m/s^2, m/s^2, m/s^2]
pos_ned = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);              % position in NED system [m, m, m]
omega_n = zeros(tInfo.TimeOfArrival(end)*trajectory.SampleRate,3);              % angular rate -> x_axiz y_axis z_axis [rad/s, rad/s, rad/s]

count = 1;
while ~isDone(trajectory)
   [pos_ned(count,:),quat_b(count),vel_n(count,:),acc_n(count,:),omega_n(count,:)] = trajectory();  % acc and vel are in navigation frame
   count = count + 1;
end


%% SOLO PER LA FASE DI TEST
% yaw_rate = deg2rad(20);  % rotazione su z-body a 20°/s
% for i = 1:size(omega_n,1)
%     if i > 5/ST
%         delta_yaw = yaw_rate * ST;
%         delta_quat = quaternion([0, 0, delta_yaw], 'eulerd', 'ZYX', 'frame');
%         new_quat = delta_quat * quat_b(i-1,:);
%         quat_b(i,:) = new_quat;
%         omega_n(i,:) = [0, 0, yaw_rate];  % solo rotazione attorno a z (yaw)
%     end
% end


%% plot result
%position
figure;
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3),"b" );
hold off


% position wrt time
figure;
subplot(3,1,1);
hold on;
grid on;
plot(time, pos_ned(:,1), "r" );
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,1),'k*');
title("North (and waypoints)")
xlabel("Time [s]")
ylabel("[m]")
subplot(3,1,2)
hold on;
grid on;
plot(time, pos_ned(:,2), "g" );
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,2),'k*');
title("East (and waypoints)")
xlabel("Time [s]")
ylabel("[m]")
subplot(3,1,3)
hold on;
grid on;
plot(time, pos_ned(:,3), "b" );
plot(tInfo.TimeOfArrival(:,1), tInfo.Waypoints(:,3),'k*');
title("Altitude (and waypoints)")
xlabel("Time [s]")
ylabel("[m]")
hold off


% velocity
figure;
plot(time, vel_n(:,1), "r" );
hold on; grid on;
plot(time, vel_n(:,2), "g" );
plot(time, vel_n(:,3), "b" );
title("Velocity")
xlabel("Time")
ylabel("[m/s]")
legend("Vel_n","Vel_e","Vel_d")
hold off

% acceleration
figure;
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("acc_n","acc_e","acc_d")
hold off

% angular velocity
figure;
plot(time, omega_n(:,1), "r" );
hold on; grid on;
plot(time, omega_n(:,2), "g" );
plot(time, omega_n(:,3), "b" );
title("Angular Velocity in Navigation Frame")
xlabel("Time")
ylabel("[rad/s]")
legend("\omega_n","\omega_e","\omega_d")
hold off

% Orientation (quaternion)
figure;
quat_c_b = compact(quat_b); % conversion from quaternion type to vector form
plot(time, quat_c_b(:,1), "r" );
hold on; grid on;
plot(time, quat_c_b(:,2), "g" );
plot(time, quat_c_b(:,3), "b" );
plot(time, quat_c_b(:,4), "m" );
title("Attitude (Quaternion)")
xlabel("Time")
legend("q_0","q_x","q_y","q_z")
hold off


%% create ideal inertial measurements

omega_b = omega_n;
acc = omega_n;


for idx=1:length(time)
    q_in = quat_b(idx,:);    
    rm = rotmat(q_in,'frame');                              % rotation matrix from navigation to body
    omega_b(idx,:) = (rm*omega_n(idx,:)')';                 % rotate omega to body frame
    eul_out = rotm2eul(rm,'ZYX'); 
    eul_out = eul_out([3 2 1]);                             % adjust order of angles returned by rotm2eul
    rpy(idx,:) = eul_out;                                   % euler angles (only to plot arrows)
    acc(idx,:) = navi2body(quat_c_b(idx,:),acc_n(idx,:));     % accelerazione in body frame
end

% plot euler angles
figure; 
hold on
plot(time,rpy(:,1)*180/pi,'r');
plot(time,rpy(:,2)*180/pi,'g');
plot(time,rpy(:,3)*180/pi,'b');
legend("roll","pitch","yaw")
title("Euler Angles from nav to body, expressed in body frame")
xlabel("Time")
ylabel("[rad]")
grid on;

% plot angular velocity in body frame
figure;
plot(time, omega_b(:,1), "r" );
hold on; grid on;
plot(time, omega_b(:,2), "g" );
plot(time, omega_b(:,3), "b" );
title("Angular Velocity in Body frame")
xlabel("Time [s]")
ylabel("[rad/s]")
legend("\omega_x","\omega_y","\omega_z")
hold off


% plot acceleration in body frame
figure;
plot(time, acc(:,1), "r" );
hold on; grid on;
plot(time, acc(:,2), "g" );
plot(time, acc(:,3), "b" );
title("Acceleration in Body frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("acc_x","acc_y","acc_z")
hold off


%% verify that acceleration and velocity are consistent with position

% integrate navigation velocity 
pos_n_vi = cumsum(vel_n).*ST + pos_ned(1,:);   % position in navigation frame, integrated by velocity
pos_n_ai = cumsum(cumsum(acc_n).*ST+vel_n(1,:)).*ST + pos_ned(1,:);    % position in navigation frame, integrated two times by acceleration
 
% same integrations as above, but using a trapezoidalic integration
pos_n_vi_t = cumtrapz(vel_n).*ST + pos_ned(1,:);    % position in navigation frame, integrated by velocity, trapezoidal integration
pos_n_ai_t = cumtrapz(cumtrapz(acc_n).*ST+vel_n(1,:)).*ST + pos_ned(1,:);

% Position wrt time
figure;
plot(time,pos_ned(:,1),'r', ...     % posizione
     time,pos_ned(:,2),'r', ...
     time,pos_ned(:,3),'r');
hold on;
plot(time,pos_n_vi(:,1),'b', ...    % position in navigation frame, integrated by velocity
     time,pos_n_vi(:,2),'b', ...
     time,pos_n_vi(:,3),'b');
plot(time,pos_n_ai(:,1),'g', ...    % position in navigation frame, integrated two times by acceleration
     time,pos_n_ai(:,2),'g', ...
     time,pos_n_ai(:,3),'g');
plot(time,pos_n_vi_t(:,1),'k:', ... % position in navigation frame, integrated by velocity, trapezoidal integration
     time,pos_n_vi_t(:,2),'k:', ...
     time,pos_n_vi_t(:,3),'k:');
plot(time,pos_n_ai_t(:,1),'m:', ... % position in navigation frame, integrated two times by acceleration, , trapezoidal integration
     time,pos_n_ai_t(:,2),'m:', ...
     time,pos_n_ai_t(:,3),'m:');
hold off
title("Position & vel & acc, Over Time")
legend("pos-n-NORTH", "pos-n-EAST", "pos-n-DOWN", ...
    "pos-n-vi-NORTH", "pos-n-vi-EAST", "pos-n-vi-DOWN", ...
    "pos-n-ai-NORTH", "pos-n-ai-EAST", "pos-n-ai-DOWN", ...
    "pos-n-vi-t-NORTH", "pos-n-vi-t-EAST", "pos-n-vi-t-DOWN", ...
    "pos-n-ai-t-NORTH", "pos-n-ai-t-EAST", "pos-n-ai-t-DOWN"); %("North","East","Down")
xlabel("Time (seconds)")
ylabel("Position (m)")
grid on


figure;
plot(time,pos_n_vi(:,1)-pos_ned(:,1),'b', ...
     time,pos_n_vi(:,2)-pos_ned(:,2),'b', ...
     time,pos_n_vi(:,3)-pos_ned(:,3),'b');
hold on;
plot(time,pos_n_ai(:,1)-pos_ned(:,1),'g', ...
     time,pos_n_ai(:,2)-pos_ned(:,2),'g', ...
     time,pos_n_ai(:,3)-pos_ned(:,3),'g');
plot(time,pos_n_vi_t(:,1)-pos_ned(:,1),'k', ...
     time,pos_n_vi_t(:,2)-pos_ned(:,2),'k', ...
     time,pos_n_vi_t(:,3)-pos_ned(:,3),'k');
plot(time,pos_n_ai_t(:,1)-pos_ned(:,1),'m', ...
     time,pos_n_ai_t(:,2)-pos_ned(:,2),'m', ...
     time,pos_n_ai_t(:,3)-pos_ned(:,3),'m');
title("Position & vel & acc, Over Time ERROR")
legend("pos-n-vi-NORTH", "pos-n-vi-EAST", "pos-n-vi-DOWN", ...
    "pos-n-ai-NORTH", "pos-n-ai-EAST", "pos-n-ai-DOWN", ...
    "pos-n-vi-t-NORTH", "pos-n-vi-t-EAST", "pos-n-vi-t-DOWN", ...
    "pos-n-ai-t-NORTH", "pos-n-ai-t-EAST", "pos-n-ai-t-DOWN"); %("North","East","Down")
xlabel("Time (seconds)")
ylabel("Position (m)")
grid on


%% verify that angular velocity is consistent with quaternion orientation             
                
% Calculate attitude variation in quaternion form
quat_i = quat_c_b;
for idx=2:length(time)
    d_angle = omega_b(idx,:) .* ST;
    d_quaternion = eul2quat([d_angle(3), d_angle(2), d_angle(1)]);
    quat_i(idx,:) = q_multi(quat_i(idx - 1,:), d_quaternion);
end

% Conversion of the quaternions to a coherent form (q and -q represent the
% same orientation --> we want them to be in the same form in order to
% compare them)
quat_crt = quat_c_b;
quat_i_conv = quat_i;
for idxx=1:length(time)
    k = quat_c_b(idxx,:);
    quat_crt(idxx,:) = conv_quat(quat_c_b(idxx,:));
    quat_i_conv(idxx,:) = conv_quat(quat_i(idxx,:));
end

figure;
hold on;
plot(time, quat_crt(:,1), 'b', 'DisplayName', 'qw real')
plot(time, quat_crt(:,2), 'g', 'DisplayName', 'qx real')
plot(time, quat_crt(:,3), 'k', 'DisplayName', 'qy real')
plot(time, quat_crt(:,4), 'm', 'DisplayName', 'qz real')
hold on;
plot(time, quat_i_conv(:,1), 'r', 'DisplayName', 'qw int')
plot(time, quat_i_conv(:,2), 'c', 'DisplayName', 'qx int')
plot(time, quat_i_conv(:,3), 'y', 'DisplayName', 'qy int')
plot(time, quat_i_conv(:,4), 'DisplayName', 'qz_int')
title("quaternions over Time")
xlabel("Time [s]")
legend('show')
hold off; grid on;

figure; 
quat_err = quat_crt - quat_i_conv;
hold on;
plot(time, quat_err(:,1), 'b', 'DisplayName', 'qw error')
plot(time, quat_err(:,2), 'g', 'DisplayName', 'qx error')
plot(time, quat_err(:,3), 'k', 'DisplayName', 'qy error')
plot(time, quat_err(:,4), 'm', 'DisplayName', 'qz error')
title("quaternion ERROR")
legend('show')
xlabel("Time [s]")
hold off; grid on;


%% Conversione da quaternion a Eulero: [yaw pitch roll] in gradi
eul_integrated = euler(quaternion(quat_i_conv), 'ZYX', 'frame');  % Da integrazione
eul_reference  = euler(quaternion(quat_crt), 'ZYX', 'frame'); % Da reference

% Plot
figure;
subplot(3,1,1);
plot(time, eul_reference(:,1), 'b', time, eul_integrated(:,1), 'r--');
ylabel('Yaw [°]'); legend('Reference','Integrated'); grid on;

subplot(3,1,2);
plot(time, eul_reference(:,2), 'b', time, eul_integrated(:,2), 'r--');
ylabel('Pitch [°]'); legend('Reference','Integrated'); grid on;

subplot(3,1,3);
plot(time, eul_reference(:,3), 'b', time, eul_integrated(:,3), 'r--');
ylabel('Roll [°]'); xlabel('Tempo [s]'); legend('Reference','Integrated'); grid on;

sgtitle('Confronto angoli di Eulero tra riferimento e integrazione');

%% position
figure;
plot3(tInfo.Waypoints(:,1),tInfo.Waypoints(:,2),tInfo.Waypoints(:,3),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3),"b" );
hold off

%% smooth accelerations and recompute everything

% signal filtering parameters
fs = 1/ST;
fc = 1;                           % cutoff frequency
order = 100;                      % filter order
% FIR design
d = fdesign.lowpass('N,Fc', order, fc, fs);
firFilter = design(d, 'window', 'Window', @hamming);
% Filter acceleration in navigation frame with a Non causal filter
acc_n_filt = filtfilt(firFilter.Numerator, 1, acc_n);

%acceleration
figure;
plot(time, acc_n(:,1), "r" );
hold on; grid on;
plot(time, acc_n(:,2), "g" );
plot(time, acc_n(:,3), "b" );
plot(time, acc_n_filt(:,1), "r:" , 'linewidth',2);
plot(time, acc_n_filt(:,2), "g:" , 'linewidth',2);
plot(time, acc_n_filt(:,3), "b:" , 'linewidth',2);
title("Acceleration in Navigation Frame")
xlabel("Time")
ylabel("[m/s^2]")
legend("a_n","a_e","a_d","a_n filt","a_e filt","a_d filt")
hold off

%recompute velocity and position (starting from 0 velocity)
vel_n_filt = cumtrapz(acc_n_filt).*ST+vel_n(1,:);
pos_n_filt = cumtrapz(vel_n_filt).*ST+pos_ned(1,:);

%plot new trajectory 
figure;
plot(time,pos_ned(:,1),'r', ...
     time,pos_ned(:,2),'r', ...
     time,pos_ned(:,3),'r');
hold on;
plot(time,pos_n_filt(:,1),'b:',  ...
     time,pos_n_filt(:,2),'b:', ...
     time,pos_n_filt(:,3),'b:', 'linewidth',2);
hold off
title("SMOOTHED - Position Over Time")
legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("[m]")
grid on

figure;
plot(time,vel_n(:,1),'r', ...
     time,vel_n(:,2),'r', ...
     time,vel_n(:,3),'r'); 
hold on;
plot(time,vel_n_filt(:,1),'b', ...
     time,vel_n_filt(:,2),'b', ...
     time,vel_n_filt(:,3),'b', 'linewidth',2);
hold off
title("SMOOTHED - Velocity Over Time")
legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("[m/s]")
grid on

% recompute acc in body frame
acc_filt_b = acc;
acc_b = acc;
for idx=1:length(time)
    q_in = quat_b(idx,:);
    rm = rotmat(q_in,'frame'); %this rot mat transforms nav to body
    acc_filt_b(idx,:) = (rm*acc_n_filt(idx,:)')'; 
    acc_b(idx,:) = (rm*acc_n(idx,:)')';
end
% Compare filtered acceleration in Body frame 
figure;
plot(time,acc_b(:,1),'r', ...
     time,acc_b(:,2),'r', ...
     time,acc_b(:,3),'r');
hold on;
plot(time,acc_filt_b(:,1),'b', ...
     time,acc_filt_b(:,2),'b', ...
     time,acc_filt_b(:,3),'b', 'linewidth',2);
hold off
title("SMOOTHED - Acceleration Over Time")
legend; %("North","East","Down")
xlabel("Time (seconds)")
ylabel("[m/s]")
grid on

%% replace raw acc with filtered acc (and pos and vel)

if use_filtered_acc
    acc = acc_filt_b;       % Accelerazione in body frame
    acc_n = acc_n_filt;     % Accelerazione in navigation frame

    vel_n = vel_n_filt;

    pos_ned = pos_n_filt; % Overwrite the pos_ned vector with the smoothed trajectory
end

% position with quiver

% Calculation of max and min values of coordinates in order to scale the axis
max_NORTH = max(pos_ned(:,1));     % NORTH component
min_NORTH = min(pos_ned(:,1));
max_EAST = max(pos_ned(:,2));      % EAST component
min_EAST = min(pos_ned(:,2));
max_DOWN = max(pos_ned(:,3));       % DOWN Component
min_DOWN = min(pos_ned(:,3));

 % Axis scale
% NORTH_range = max_NORTH - min_NORTH;
% EAST_range = max_EAST - min_EAST;
% DOWN_range = max_DOWN - min_DOWN;
% 
% quiver_scale_factor = min([NORTH_range, EAST_range, DOWN_range])/max([NORTH_range, EAST_range, DOWN_range])*10;
% scale_lat = 1; 
% scale_lon = 1; 
% scale_alt = 1;
% 
% figure;
% hold on;
% grid on;
% 
% plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), 'b');
% 
% plot3(waypoints_ned(:,2), waypoints_ned(:,3), waypoints_ned(:,4), 'k*', 'MarkerSize', 10);
% 
% % Arrow components
% x_axis_ned_x = cos(rpy(:,2)) .* cos(rpy(:,3));
% x_axis_ned_y = cos(rpy(:,2)) .* sin(rpy(:,3));
% x_axis_ned_z = -sin(rpy(:,2));
% x_axis_ned = [x_axis_ned_x, x_axis_ned_y, x_axis_ned_z];
% 
% undersample = 1000;
% quiver3(pos_ned(1:undersample:end,1), pos_ned(1:undersample:end,2), pos_ned(1:undersample:end,3), ...
%     x_axis_ned(1:undersample:end,1), ...
%     x_axis_ned(1:undersample:end,2), ...
%     x_axis_ned(1:undersample:end,3));
% 
% xlabel('North [m]');
% ylabel('East [m]');
% zlabel('Down [m]');
% title('Trajectory with Navigation Vectors in NED Coordinates');
% view(3);
% legend('Trajectory','Navigation Vectors');
% hold off;
figure;
hold on; grid on; axis equal;

plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), 'b');
plot3(waypoints_ned(:,2), waypoints_ned(:,3), waypoints_ned(:,4), 'k*', 'MarkerSize', 10);

undersample = 1000;
scale = 500;

for i = 1:undersample:length(pos_ned)
    dir_x = rotateframe(conj(quat_b(i,:)), [1 0 0]);  % x_body in NED
    quiver3(pos_ned(i,1), pos_ned(i,2), pos_ned(i,3), ...
            dir_x(1)*scale, dir_x(2)*scale, dir_x(3)*scale, ...
            'r', 'LineWidth', 1.5);
end

xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title('Trajectory with x\_body Direction in NED');
legend('Trajectory','Waypoints','x\_body');
view(3);


% position with axis
% 
% figure;
% hold on;
% grid on;
% 
% plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), 'b');
% 
% phi = rpy(:,1); 
% theta = rpy(:,2); 
% psi = rpy(:,3);
% 
% y_axis_ned_x = -cos(phi) .* sin(psi) + cos(psi) .* sin(phi) .* sin(theta);
% y_axis_ned_y = cos(phi) .* cos(psi) + sin(psi) .* sin(phi) .* sin(theta);
% y_axis_ned_z = sin(phi) .* cos(theta);
% y_axis_ned = [y_axis_ned_x, y_axis_ned_y, y_axis_ned_z];
% 
% z_axis_ned_x = sin(phi) .* sin(psi) + cos(psi) .* cos(phi) .* sin(theta);
% z_axis_ned_y = -sin(phi) .* cos(psi) + sin(psi) .* cos(phi) .* sin(theta);
% z_axis_ned_z = cos(phi) .* cos(theta);
% z_axis_ned = [z_axis_ned_x, z_axis_ned_y, z_axis_ned_z];
% 
% quiver3(pos_ned(1:undersample:end,1), pos_ned(1:undersample:end,2), pos_ned(1:undersample:end,3), ...
%     x_axis_ned(1:undersample:end,1), ...
%     x_axis_ned(1:undersample:end,2), ...
%     x_axis_ned(1:undersample:end,3), ...
%     1/3);
% 
% quiver3(pos_ned(1:undersample:end,1), pos_ned(1:undersample:end,2), pos_ned(1:undersample:end,3), ...
%     y_axis_ned(1:undersample:end,1), ...
%     y_axis_ned(1:undersample:end,2), ...
%     y_axis_ned(1:undersample:end,3), ...
%     1/3);
% 
% quiver3(pos_ned(1:undersample:end,1), pos_ned(1:undersample:end,2), pos_ned(1:undersample:end,3), ...
%     z_axis_ned(1:undersample:end,1), ...
%     z_axis_ned(1:undersample:end,2), ...
%     z_axis_ned(1:undersample:end,3), ...
%     1/10);
% 
% title('Trajectory with Navigation Axes in NED Coordinates');
% xlabel('Longitude [m]');
% ylabel('Latitude [m]');
% zlabel('Altitude [m]');
% view(3);
% legend({'Trajectory', 'X Axis', 'Y Axis', 'Z Axis'});
% hold off;

figure;
hold on; grid on; axis equal;

plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), 'b');

undersample = 1000;
scale = 500;

for i = 1:undersample:length(pos_ned)
    x_dir = rotateframe(conj(quat_b(i,:)), [1 0 0]);  % x_body → NED
    y_dir = rotateframe(conj(quat_b(i,:)), [0 1 0]);  % y_body → NED
    z_dir = rotateframe(conj(quat_b(i,:)), [0 0 1]);  % z_body → NED

    pos = pos_ned(i,:);
    quiver3(pos(1), pos(2), pos(3), x_dir(1)*scale, x_dir(2)*scale, x_dir(3)*scale, 'r', 'LineWidth', 1.5);
    quiver3(pos(1), pos(2), pos(3), y_dir(1)*scale, y_dir(2)*scale, y_dir(3)*scale, 'g', 'LineWidth', 1.5);
    quiver3(pos(1), pos(2), pos(3), z_dir(1)*scale, z_dir(2)*scale, z_dir(3)*scale, 'b', 'LineWidth', 1.5);
end

xlabel('North [m]'); ylabel('East [m]'); zlabel('Down [m]');
title('Trajectory with Body Axes in NED Frame');
legend('Trajectory', 'x\_body', 'y\_body', 'z\_body');
view(3);


%% create accelerometer measurments (by adding gravity) --> NOTA: questo lo rimuovo da qui e lo faccio direttamente da Simulink
%add gravity to acc in body frame
%define gravity value
% g_n = [0 0 9.81]';   % Gravity in navigation frame
% acc_f = acc;
% for idx=1:length(time)
%     q_in = quat_b(idx,:);
%     rm = rotmat(q_in,'frame'); %this rot mat transforms nav to body
%     acc(idx,:) = acc(idx,:)+(rm*g_n)'; %add gravity with + sign
% end


%% remove NaN
% acc
for i = 2:size(acc, 1)
    for j = 1:size(acc, 2)
        if isnan(acc(i, j))
            acc(i, j) = acc(i-1, j);
        end
    end
end

% omega_b
for i = 2:size(omega_b, 1)
    for j = 1:size(omega_b, 2)
        if isnan(omega_b(i, j))
            omega_b(i, j) = omega_b(i-1, j);
        end
    end
end

% 


%% Freccia orientata lungo x_body per ogni posizione
scale = 100;  % lunghezza freccia
figure(23);
plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3), 'b'); hold on; grid on; axis equal;
for i = 1:100:length(time)
    r = rotateframe(conj(quat_b(i)), [1 0 0]);  % x_body → nav
    quiver3(pos_ned(i,1), pos_ned(i,2), pos_ned(i,3), ...
            r(1)*scale, r(2)*scale, r(3)*scale, ...
            'r', 'LineWidth', 1.5);
end