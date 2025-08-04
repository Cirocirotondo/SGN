%% H calculation
% y_pred = h(x_pred) is the function that gives the measurement
% corresponding to a given state vector
% H = dh/dx is the jacobian of h relative to the state variable x


syms north east down            % pos
syms vel_n vel_e vel_d          % vel
syms acc_n acc_e acc_d          % acc
syms q_w q_x q_y q_z            % quaternion
syms w_x w_y w_z                % omega
syms byas_gx byas_gy byas_gz    % byass gyro
syms dt

% Measure = 12-dimensional vector
% measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_n, acc_e, acc_d), Mag (n, e, d), gyro (wx, wy, wz)]

% State = 19-dimensional vector
% [north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]
x = [north; east; down; vel_n; vel_e; vel_d; acc_n; acc_e; acc_d; q_w; q_x; q_y; q_z; w_x; w_y; w_z; byas_gx; byas_gy; byas_gz];

% UWB measure
UWB_n = north;
UWB_e = east;
UWB_d = down;

% accelerometer measure
accel_n = acc_n;
accel_e = acc_e;
accel_d = acc_d;

% magnetometer measure
mag_sym_1 = 0.24*( q_w^2 +  q_x^2 -  q_y^2 -  q_z^2)+ 2*0.40*( q_w* q_y +  q_x* q_z);
mag_sym_2 = 2*0.40 *( q_y* q_z -  q_w* q_x) + 2*0.24*( q_w* q_z +  q_x* q_y);
mag_sym_3 = 0.40*( q_w^2 -  q_x^2 -  q_y^2 +  q_z^2) +2*0.24 *( q_z* q_x -  q_w* q_y);

% gyroscope measure
gyro_x = w_x + byas_gx;
gyro_y = w_y + byas_gy;
gyro_z = w_z + byas_gz;
 
% Jacobian computation
Jacob_UWB_n = jacobian(UWB_n,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_UWB_e = jacobian(UWB_e,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_UWB_d = jacobian(UWB_d,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accel_n = jacobian(accel_n,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accel_e = jacobian(accel_e,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_accel_d = jacobian(accel_d,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_magx = jacobian(mag_sym_1,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_magy = jacobian(mag_sym_2,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_magz = jacobian(mag_sym_3,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_gyrox = jacobian(gyro_x,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_gyroy = jacobian(gyro_y,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);
Jacob_gyroz = jacobian(gyro_z,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);

%PITOT
Jacob_pitot = jacobian(vel_n,[north, east, down, vel_n, vel_e, vel_d, acc_n, acc_e, acc_d, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]);


Jacob_H = [
        Jacob_UWB_n;
        Jacob_UWB_e;
        Jacob_UWB_d;
        Jacob_accel_n;
        Jacob_accel_e;
        Jacob_accel_d;
        Jacob_magx;
        Jacob_magy;
        Jacob_magz;
        Jacob_gyrox;
        Jacob_gyroy;
        Jacob_gyroz;
        Jacob_pitot
    ];

H_func = matlabFunction(Jacob_H, 'File','KF_utilities/H_matrix', 'Vars', {x});