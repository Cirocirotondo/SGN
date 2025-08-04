%% H calculation
% y_pred = h(x_pred) is the function that gives the measurement
% corresponding to a given state vector
% H = dh/dx is the jacobian of h relative to the state variable x

syms roll pitch yaw             % orientation (from nav to body)
syms w_x w_y w_z                % omega (body frame)
syms byas_gx byas_gy byas_gz    % byass gyro
syms dt

% Measure = 12-dimensional vector
% measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_x, acc_y, acc_z), Mag (n, e, d), gyro (wx, wy, wz)]

% State = 9-dimensional vector
x =  [roll; pitch; yaw; w_x; w_y; w_z; byas_gx; byas_gy; byas_gz];

% mag
mag_n = [0.24 0 0.40]'; % approx Gauss field, no declination 

%compute Cnb
Cz=[cos(yaw)     sin(yaw)     0; ... 
    -sin(yaw)    cos(yaw)     0; ...
    0               0               1];
Cy=[cos(pitch)     0               -sin(pitch); ...
    0               1               0; ...
    sin(pitch)     0               cos(pitch)];
Cx=[1               0               0;...
    0               cos(roll)     sin(roll); ...
    0               -sin(roll)    cos(roll)];
Cnb = (Cx*Cy*Cz);
mag_b = Cnb*mag_n;
mag_sym_1 = mag_b(1);
mag_sym_2 = mag_b(2);
mag_sym_3 = mag_b(3);

% gyro
gyro_x = w_x + byas_gx;
gyro_y = w_y + byas_gy;
gyro_z = w_z + byas_gz;

state = [roll, pitch, yaw, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz];
 
% Jacobian computation

Jacob_magx = jacobian(mag_sym_1,state);
Jacob_magy = jacobian(mag_sym_2,state);
Jacob_magz = jacobian(mag_sym_3,state);
Jacob_gyrox = jacobian(gyro_x,state);
Jacob_gyroy = jacobian(gyro_y,state);
Jacob_gyroz = jacobian(gyro_z,state);

Jacob_H = [
        Jacob_magx;
        Jacob_magy;
        Jacob_magz;
        Jacob_gyrox;
        Jacob_gyroy;
        Jacob_gyroz;
    ];

H_func = matlabFunction(Jacob_H, 'File','KF_utilities/H_matrix_only_attitude', 'Vars', {x});