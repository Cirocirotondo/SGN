%% Script Matlab for the trjectory generation 
%  It provides the waypoints for the trajectory_generation_from_wp_geod.m script



ST = 1/100; % Sample time
dt = ST;

%% First trajectory

if traj_sel==1 % movimenti vari

    t_max = 200; % Simulation time
    time = (0:ST:(t_max))'; % time vector

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 30;   % Takeoff and climb
    t2_end = 70;   % Turn
    t3_end = 130;  % Straight cruise
    t4_end = 170;  % Turn and descend
    t5_end = t_max; % Final approach

    % Parameters
    v_climb = 10;         % m/s horizontal speed during climb
    climb_rate = -2;      % m/s (negative down = climbing)
    turn_radius = 100;    % meters
    v_cruise = 20;        % m/s cruise speed
    desc_rate = 1.5;      % m/s descent rate (positive down)
    angle_rate = v_cruise / turn_radius; % rad/s for constant turn

    % Initialize

    % starting position
    n = 0; e = 0; d = 0;
    heading = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: Takeoff and climb straight
            pos_ned_n(i) = n + v_climb * t;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d + climb_rate * t;
            yaw(i) = 0;
            pitch(i) = atan2(-climb_rate, v_climb); % pitch based on climb angle
        elseif t <= t2_end
            % Phase 2: Turn at constant radius
            dt = t - t1_end;
            theta = angle_rate * dt;
            pos_ned_n(i) = pos_ned_n(t1_end/ST+1) + turn_radius * sin(theta);
            pos_ned_e(i) = pos_ned_e(t1_end/ST+1) + turn_radius * (1 - cos(theta));
            pos_ned_d(i) = pos_ned_d(t1_end/ST+1);
            yaw(i) = theta;
            pitch(i) = 0;
            roll(i) = 0.3;
        elseif t <= t3_end
            % Phase 3: Straight cruise
            dt = t - t2_end;
            heading = angle_rate * (t2_end - t1_end); % constant from end of turn
            pos_ned_n(i) = pos_ned_n(t2_end/ST+1) + v_cruise * dt * cos(heading);
            pos_ned_e(i) = pos_ned_e(t2_end/ST+1) + v_cruise * dt * sin(heading);
            pos_ned_d(i) = pos_ned_d(t2_end/ST+1);
            yaw(i) = heading;
            pitch(i) = 0;
        elseif t <= t4_end
            % Phase 4: Turn and descend
            dt = t - t3_end;
            theta = angle_rate * dt;
            pos_ned_n(i) = pos_ned_n(t3_end/ST+1) + turn_radius * (cos(0) - cos(theta));
            pos_ned_e(i) = pos_ned_e(t3_end/ST+1) + turn_radius * sin(theta);
            pos_ned_d(i) = pos_ned_d(t3_end/ST+1) + desc_rate * dt;
            yaw(i) = pi + theta;
            pitch(i) = atan2(desc_rate, v_cruise);
            roll(i) = -0.3;
        else
            % Phase 5: Final straight approach
            dt = t - t4_end;
            pos_ned_n(i) = pos_ned_n(t4_end/ST+1) + v_climb * dt;
            pos_ned_e(i) = pos_ned_e(t4_end/ST+1);
            pos_ned_d(i) = pos_ned_d(t4_end/ST+1) + desc_rate * dt;
            yaw(i) = pi;
            pitch(i) = atan2(desc_rate, v_climb);
        end
    end

    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];

    euler_angles = [yaw, pitch, roll];

    total_trajectory = [pos_ned, euler_angles];
    
   

    % Waypoints generation

    % Come waypoints prendiamo 45 punti della traiettoria, equidistanziati
    % nel tempo
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));

    % wp: Time [s], Position (x,y,z) [m], Orientation (yaw pitch roll) [DEG]
    waypoints_ned = [time(ind), total_trajectory(ind, :)];



%% Second trajectory

elseif traj_sel==2

    waypoints_ned = [ 0   0   0         0         0         0         0;...        
        1   0           0         0         0         0         0;...
    6.9444   27.0353  -11.0895    -1.2691  -17.8689    4.6448    4.4314;...
   13.8889   38.0952  -25.0973    -2.4873  -51.7070   -1.8029   -1.8645;...
   20.8333   46.6974  -48.0545    -3.6128  -69.4588   -6.6726   -6.1263;...
   27.7778   39.6313  -71.0117    -4.7212 -107.1080  -10.8735   -9.1531;...
   34.7222   27.9570  -79.9611    -5.9627 -142.5267  -13.1720   -8.7144;...
   41.6667  -44.5469  -82.2957    -6.4027 -178.1557   -8.0442   -2.0766;...
   48.6111  -67.5883  -74.9027    -7.0973  162.2108   -6.5044    3.2926;...
   55.5556  -81.7204  -50.3891    -8.6248  119.9635   -8.1054    3.0096;...
   62.5000  -86.3287   -3.3074   -10.7885   95.5902   -7.6819   -1.3601;...
   69.4444  -85.4071   47.2763   -13.3720   88.9562   -3.7470   -6.6140;...
   76.3889  -69.7389   76.4591   -15.2714   61.7688    1.5208  -12.3492;...
   83.3333  -46.3902   86.9650   -15.9091   24.2255    9.3477  -13.2543;...
   90.2778  -19.3548   89.2996   -15.7643    4.9355   14.8030   -8.1259;...
   97.2222    2.1505   88.9105   -14.7595   -1.0366   14.6919   -6.5567;...
  104.1667    8.6022   88.9105   -13.5347         0   11.9302   -7.4452;...
  111.1111   12.2888   86.9650   -13.3567  -27.8217    3.0853   -7.7571;...
  118.0556   14.1321   85.4086   -13.6538  -40.1763   -8.4237   -8.6308;...
  125.0000   15.0538   83.0739   -13.8232  -68.4570  -13.3703   -4.1111;...
  131.9444   14.1321   78.7938   -14.4416 -102.1521  -12.4273    4.4139;...
  138.8889    7.6805   78.4047   -14.8874 -176.5486   -8.7867    9.2389;...
  145.8333   -0.9217   76.4591   -14.2209 -167.2560   -0.4272    9.7053;...
  152.7778   -1.5361   70.2335   -13.3239  -95.6365    5.8486    6.1606;...
  159.7222    0.3072   64.3969   -12.2226  -72.4727    6.2868   -0.1808;...
  166.6667    6.7588   62.4514   -11.1399  -16.7810    8.5828   -0.7664;...
  173.6111   11.0599   60.1167   -10.8808  -28.4932   12.3066    4.7437;...
  180.5556   15.9754   55.4475   -10.5200  -43.5283   12.0021    8.3825;...
  187.5000   16.8971   49.2218    -9.3512  -81.5790    6.3497    7.3499;...
  194.4444   15.0538   42.6070    -7.4707 -105.5714   -0.8404    1.7566;...
  201.3889   14.1321   35.2140    -5.0758  -97.1062   -6.5643   -2.5193;...
  208.3333   11.0599   29.3774    -2.7465 -117.7610   -9.9435   -0.2147;...
  215.2778    8.6022   25.8755    -1.8824 -125.0622  -11.0863    2.4769;...
  222.2222    6.7588   20.8171    -2.2549 -110.0222  -11.5590    4.6307;...
  229.1667    2.1505   11.8677    -1.6681 -117.2452   -8.1527    5.6081;...
  236.1111    0.3072    5.6420    -0.5689 -106.4931   -2.8508    3.2170;...
  243.0556    0.3072    4.8638   -0.9765  -90.0000    4.6519   -1.9190;...
  250.0000   -0.6144    1.7510    0.000   -106.4931   14.8082   -9.5876];
  

   

    %% Plot
    
    % Traiettoria, in NED, per mostrare il suo andamento in un sistema cartesiano
    figure;
    plot3(waypoints_ned(:,2), waypoints_ned(:,3), waypoints_ned(:,4), 'ko');
    title("Waypoints")
    xlabel("North")
    ylabel("East")
    zlabel("Down")
    grid on
    axis equal
    set(gca,"XDir","reverse")
    set(gca,"ZDir","reverse")

elseif traj_sel == 3  % fermo e poi linea retta
    t_max = 200; % Simulation time
    time = (0:ST:(t_max))'; % time vector

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;    % stay still (to find gyro bias)
    t2_end = t_max; % Final approach

    % Parameters
    v_cruise = 20;        % m/s cruise speed

    % Initialize
    n = 0; e = 0; d = 0;
    heading = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0; % pitch based on climb angle
        elseif t <= t2_end
            % Phase 2: travel at constant speed
            pos_ned_n(i) = pos_ned_n(i-1) + v_cruise*ST;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        end
    end

    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];

    euler_angles = [yaw, pitch, roll];

    total_trajectory = [pos_ned, euler_angles];


    % Waypoints generation

    % Come waypoints prendiamo 45 punti della traiettoria, equidistanziati
    % nel tempo
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));

    % wp: Time [s], Position (x,y,z) [m], Orientation (yaw pitch roll) [DEG]
    waypoints_ned = [time(ind), total_trajectory(ind, :)];

elseif traj_sel == 4  % fermo poi acceleraz a jerk costante e poi linea retta
    t_max = 200; % Simulation time
    time = (0:ST:(t_max))'; % time vector

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerck
    t3_end = 45;    % constant negative jerck
    t4_end = t_max; % Final approach

    % Parameters
    v_cruise = 100;        % m/s cruise speed

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    heading = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
            yaw(i) = 0;
            pitch(i) = 0; % pitch based on climb angle
        elseif t <= t2_end
            % Phase 2: positive constant jerk
            a_n = a_n + jerk*dt;
            acc_n_backup(i) = a_n;
            v_n = v_n + a_n*dt;
            pos_ned_n(i) = pos_ned_n(i-1) + v_n*dt;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        elseif t <= t3_end
            % Phase 2: negative constant jerk
            a_n = a_n - jerk*dt;
            acc_n_backup(i) = a_n;
            v_n = v_n + a_n*dt;
            pos_ned_n(i) = pos_ned_n(i-1) + v_n*dt;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        elseif t <= t4_end
            % Phase 3: travel at constant speed
            pos_ned_n(i) = pos_ned_n(i-1) + v_cruise*ST;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        end
    end

    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];

    euler_angles = [yaw, pitch, roll];

    total_trajectory = [pos_ned, euler_angles];


    % Waypoints generation

    % Come waypoints prendiamo 45 punti della traiettoria, equidistanziati
    % nel tempo
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));

    % wp: Time [s], Position (x,y,z) [m], Orientation (yaw pitch roll) [DEG]
    waypoints_ned = [time(ind), total_trajectory(ind, :)];

elseif traj_sel == 5
    t_max = 100; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    
    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);
    
    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerck
    t3_end = 45;    % constant negative jerck
    t4_end = t_max; % Final approach
    
    % Parameters
    v_cruise = 100;        % m/s cruise speed
    
    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    heading = 0;
    
    % Virata parameters
    turn_start = 60;        % Inizio virata [s]
    turn_duration = 30;      % Durata virata [s]
    turn_angle = pi/2;       % Virata di 90 gradi
    radius = 500;            % Raggio della virata [m]
    
    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            acc_n_backup(i) = a_n;
            v_n = v_n + a_n*dt;
            pos_ned_n(i) = pos_ned_n(i-1) + v_n*dt;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            acc_n_backup(i) = a_n;
            v_n = v_n + a_n*dt;
            pos_ned_n(i) = pos_ned_n(i-1) + v_n*dt;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = 0;
            pitch(i) = 0;
            roll(i) = 0;
        elseif t <= t4_end
            if t < turn_start
                % Phase 4a: straight flight
                pos_ned_n(i) = pos_ned_n(i-1) + v_cruise*dt;
                pos_ned_e(i) = pos_ned_e(i-1);
                yaw(i) = 0;
            else
                % Phase 4b: turning
                theta = (t - turn_start) / turn_duration * turn_angle;
                theta = min(theta, turn_angle);  % clamp angle
                pos_ned_n(i) = pos_ned_n(i-1) + v_cruise * cos(theta) * dt;
                pos_ned_e(i) = pos_ned_e(i-1) + v_cruise * sin(theta) * dt;
                yaw(i) = rad2deg(theta);
            end
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            pitch(i) = 0;
            roll(i) = 0;
        end
    end
    
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];
    
    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];


elseif traj_sel == 6
    t_max = 110; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    dt = ST; % time step

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerk
    t3_end = 45;    % constant negative jerk
    t4a_end = 55;   % fine parabola di salita
    t4_end  = 65;   % fine anti-parabola
    t5_end = 70;    % linear
    t6_end = 100;    % virata
    t7_end = t_max; % Final approach

    % Parameters
    v_cruise = 100;         % m/s cruise speed
    acc_ascent = 2;         % m/s^2 ascent acceleration
    h_max = 100;            % Altezza da guadagnare (positiva, d scende in NED)
    turn_angle = pi/2;      % Virata di 90 gradi
    turn_duration = t6_end - t5_end;

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    vel_d = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4a_end
            % Phase 4a: climb (parabolic ascent)
            vel_d = vel_d + acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;
            
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4_end
            % Phase 4 (climb smooth using cubic polynomial)
            vel_d = vel_d - acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;

            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t5_end
            % Phase 5: straight flight at constant altitude
            n = n + v_cruise * dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t6_end
            % Phase 6: turning
            theta = (t - t5_end) / turn_duration * turn_angle;
            theta = min(theta, turn_angle);  % clamp
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = rad2deg(theta);
        elseif t <= t7_end
            % Phase 7: straight after turn
            dn = v_cruise * cos(turn_angle) * dt;
            de = v_cruise * sin(turn_angle) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = rad2deg(turn_angle);
        end
    end

    % Output
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];

    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];

elseif traj_sel == 7
    t_max = 140; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    dt = ST; % time step

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerk
    t3_end = 45;    % constant negative jerk
    t4a_end = 55;   % fine parabola di salita
    t4_end  = 65;   % fine anti-parabola
    t5_end = 70;    % linear
    t6_end = 100;   % virata
    t7_end = 110;   % linear
    t8_end = t_max;   % seconda virata

    % Parameters
    v_cruise = 100;         % m/s cruise speed
    acc_ascent = 2;         % m/s^2 ascent acceleration
    h_max = 100;            % Altezza da guadagnare (positiva, d scende in NED)
    turn_angle = pi/2;      % Virata di 90 gradi
    turn_angle2 = 3*pi/2;   % 270°
    turn_duration = t6_end - t5_end;
    turn_duration2 = t8_end - t7_end;
    v_climb = 10;

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    vel_d = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4a_end
            % Phase 4a: climb (parabolic ascent)
            vel_d = vel_d + acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;
            
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4_end
            % Phase 4 (climb smooth using cubic polynomial)
            vel_d = vel_d - acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;

            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t5_end
            % Phase 5: straight flight at constant altitude
            n = n + v_cruise * dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t6_end
            % Phase 6: turning
            theta = (t - t5_end) / turn_duration * turn_angle;
            theta = min(theta, turn_angle);  % clamp
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            % d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(theta);
        elseif t <= t7_end
            % Phase 7: straight after turn
            dn = v_cruise * cos(turn_angle) * dt;
            de = v_cruise * sin(turn_angle) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(turn_angle);
        elseif t <= t8_end
            theta = turn_angle - (t - t7_end) / turn_duration2 * turn_angle2;
            theta = max(theta, turn_angle - turn_angle2);
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = -rad2deg(mod(theta, 2*pi));

        end
    end
    

    % Output
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];

    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];

elseif traj_sel == 8
    t_max = 200; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    dt = ST; % time step

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerk
    t3_end = 45;    % constant negative jerk
    t4a_end = 55;   % fine parabola di salita
    t4_end  = 65;   % fine anti-parabola
    t5_end = 70;    % linear
    t6_end = 100;   % virata
    t7_end = 110;   % linear
    t8_end = 180;   % seconda virata
    t9_end = t_max; % linear

    % Parameters
    v_cruise = 100;         % m/s cruise speed
    acc_ascent = 2;         % m/s^2 ascent acceleration
    h_max = 100;            % Altezza da guadagnare (positiva, d scende in NED)
    turn_angle = pi/2;      % Virata di 90 gradi
    turn_angle2 = 3*pi/2;   % 270°
    turn_duration = t6_end - t5_end;
    turn_duration2 = t8_end - t7_end;
    v_climb = 10;

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    vel_d = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4a_end
            % Phase 4a: climb (parabolic descent)
            vel_d = vel_d + acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;
            
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4_end
            % Phase 4 (climb smooth using cubic polynomial)
            vel_d = vel_d - acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;

            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t5_end
            % Phase 5: straight flight at constant altitude
            n = n + v_cruise * dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t6_end
            % Phase 6: turning
            theta = (t - t5_end) / turn_duration * turn_angle;
            theta = min(theta, turn_angle);  % clamp
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            % d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(theta);
        elseif t <= t7_end
            % Phase 7: straight after turn
            dn = v_cruise * cos(turn_angle) * dt;
            de = v_cruise * sin(turn_angle) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(turn_angle);
        elseif t <= t8_end
            theta = turn_angle - (t - t7_end) / turn_duration2 * turn_angle2;
            theta = max(theta, turn_angle - turn_angle2);
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = -rad2deg(mod(theta, 2*pi));

        elseif t <= t9_end
            % Phase 9: straight after turn
            dn = -v_cruise * dt;
            de = 0;
            n = n + dn;
            e = e + de;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(turn_angle);

        end
    end

    % Output
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];

    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];



elseif traj_sel == 9
    t_max = 200; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    dt = ST; % time step

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerk
    t3_end = 45;    % constant negative jerk
    t4a_end = 55;   % fine parabola di salita
    t4_end  = 65;   % fine anti-parabola
    t5_end = 70;    % linear
    t6_end = 100;   % virata
    t7_end = 110;   % linear
    t8_end = 180;   % seconda virata
    t9_end = t_max; % linear

    % Parameters
    v_cruise = 100;         % m/s cruise speed
    acc_ascent = 2;         % m/s^2 ascent acceleration
    h_max = 100;            % Altezza da guadagnare (positiva, d scende in NED)
    turn_angle = pi/2;      % Virata di 90 gradi
    turn_angle2 = 3*pi/2;   % 270°
    turn_duration = t6_end - t5_end;
    turn_duration2 = t8_end - t7_end;
    v_climb = 10;

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    vel_d = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4a_end
            % Phase 4a: climb (parabolic descent)
            vel_d = vel_d + acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;
            
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4_end
            % Phase 4 (climb smooth using cubic polynomial)
            vel_d = vel_d - acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;

            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t5_end
            % Phase 5: straight flight at constant altitude
            n = n + v_cruise * dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t6_end
            % Phase 6: turning
            theta = (t - t5_end) / turn_duration * turn_angle;
            theta = min(theta, turn_angle);  % clamp
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(theta);
        elseif t <= t7_end
            % Phase 7: straight after turn
            dn = v_cruise * cos(turn_angle) * dt;
            de = v_cruise * sin(turn_angle) * dt;
            n = n + dn;
            e = e + de;
            d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(turn_angle);
        elseif t <= t8_end
            theta = turn_angle - (t - t7_end) / turn_duration2 * turn_angle2;
            theta = max(theta, turn_angle - turn_angle2);
            dn = v_cruise * cos(theta) * dt;
            de = v_cruise * sin(theta) * dt;
            n = n + dn;
            e = e + de;
            d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            yaw(i) = -rad2deg(mod(theta, 2*pi));

        elseif t <= t9_end
            % Phase 9: straight after turn
            dn = -v_cruise * dt;
            de = 0;
            n = n + dn;
            e = e + de;
            d = d + v_climb*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
            yaw(i) = -rad2deg(turn_angle);

        end
    end

    % Output
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];

    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];


elseif traj_sel == 10
    t_max = 100; % Simulation time
    time = (0:ST:(t_max))'; % time vector
    dt = ST; % time step

    % Preallocate NED vectors
    pos_ned_n = zeros(length(time),1);
    pos_ned_e = zeros(length(time),1);
    pos_ned_d = zeros(length(time),1);
    yaw = zeros(length(time),1);
    pitch = zeros(length(time),1);
    roll = zeros(length(time),1);

    % Trajectory phases
    t1_end = 5;     % stay still (to find gyro bias)
    t2_end = 25;    % constant jerk
    t3_end = 45;    % constant negative jerk
    t4a_end = 70;   % fine parabola di salita
    t4_end  = 95;       % fine anti-parabola
    t5_end = t_max;    % linear

    % Parameters
    v_cruise = 100;         % m/s cruise speed
    acc_ascent = 2;         % m/s^2 ascent acceleration
    h_max = 100;            % Altezza da guadagnare (positiva, d scende in NED)

    % Initialize
    n = 0; e = 0; d = 0;
    a_n = 0; v_n = 0;
    acc_n_backup = zeros(length(time)+1,1);
    jerk = 0.25;
    vel_d = 0;

    for i = 1:length(time)
        t = time(i);
        if t <= t1_end
            % Phase 1: stay still
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = 0;
        elseif t <= t2_end
            % Phase 2: accelerate with positive jerk
            a_n = a_n + jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t3_end
            % Phase 3: decelerate with negative jerk
            a_n = a_n - jerk*dt;
            v_n = v_n + a_n*dt;
            n = n + v_n*dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4a_end
            % Phase 4a: climb (parabolic descent)
            vel_d = vel_d + acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;
            
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t4_end
            % Phase 4 (climb smooth using cubic polynomial)
            vel_d = vel_d - acc_ascent * dt;
            dd = vel_d * dt;    % delta down position
            d = d + dd;

            dn = v_cruise * dt;
            n = n + dn;

            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        elseif t <= t5_end
            % Phase 5: straight flight at constant altitude
            n = n + v_cruise * dt;
            pos_ned_n(i) = n;
            pos_ned_e(i) = e;
            pos_ned_d(i) = d;
            acc_n_backup(i) = a_n;
        end
    end

    % Output
    pos_ned = [pos_ned_n pos_ned_e pos_ned_d];
    euler_angles = [yaw, pitch, roll];
    total_trajectory = [pos_ned, euler_angles];

    % Waypoints generation
    num_waypoints = 45;
    ind = round(linspace(1, length(time), num_waypoints));
    waypoints_ned = [time(ind), total_trajectory(ind, :)];

end







%position
figure;
plot3(waypoints_ned(:,2),waypoints_ned(:,3),waypoints_ned(:,4),"r*")
title("Trajectory")
xlabel("North")
ylabel("East")
zlabel("Down")
grid on
axis equal
hold on
plot3(pos_ned(:,1), pos_ned(:,2), pos_ned(:,3),"b" );
hold off
