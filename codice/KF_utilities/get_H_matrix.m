%% H calculation
% y_pred = h(x_pred) is the function that gives the measurement
% corresponding to a given state vector
% H = dh/dx is the jacobian of h relative to the state variable x

syms north east down            % pos (NED)
syms vel_n vel_e vel_d          % vel (nav frame)
syms acc_x acc_y acc_z          % acc (body frame)
syms q_w q_x q_y q_z            % quaternion (from nav to body)
syms w_x w_y w_z                % omega (body frame)
syms byas_gx byas_gy byas_gz    % byass gyro

% Measure = 12-dimensional vector
% measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_x, acc_y, acc_z), Mag (n, e, d), gyro (wx, wy, wz)]

% State = 19-dimensional vector
% [north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]
x = [north; east; down; vel_n; vel_e; vel_d; acc_x; acc_y; acc_z; q_w; q_x; q_y; q_z; w_x; w_y; w_z; byas_gx; byas_gy; byas_gz];

% UWB measure
UWB_n = north;
UWB_e = east;
UWB_d = down;

% accelerometer measure
accel_x = acc_x;
accel_y = acc_y;
accel_z = acc_z;

% magnetometer measure
mag_sym_1 = 0.24*( q_w^2 +  q_x^2 -  q_y^2 -  q_z^2)+ 2*0.40*( q_w* q_y +  q_x* q_z);
mag_sym_2 = 2*0.40 *( q_y* q_z -  q_w* q_x) + 2*0.24*( q_w* q_z +  q_x* q_y);
mag_sym_3 = 0.40*( q_w^2 -  q_x^2 -  q_y^2 +  q_z^2) +2*0.24 *( q_z* q_x -  q_w* q_y);

% gyroscope measure
gyro_x = w_x + byas_gx;
gyro_y = w_y + byas_gy;
gyro_z = w_z + byas_gz;
 

% Jacobian computation
Jacob_UWB_n = jacobian(UWB_n, x);
Jacob_UWB_e = jacobian(UWB_e, x);
Jacob_UWB_d = jacobian(UWB_d, x);
Jacob_accel_n = jacobian(accel_x, x);
Jacob_accel_e = jacobian(accel_y, x);
Jacob_accel_d = jacobian(accel_z, x);
Jacob_magx = jacobian(mag_sym_1, x);
Jacob_magy = jacobian(mag_sym_2, x);
Jacob_magz = jacobian(mag_sym_3, x);
Jacob_gyrox = jacobian(gyro_x, x);
Jacob_gyroy = jacobian(gyro_y, x);
Jacob_gyroz = jacobian(gyro_z, x);

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
    ];

H_func = matlabFunction(Jacob_H, 'File','KF_utilities/H_matrix', 'Vars', {x});