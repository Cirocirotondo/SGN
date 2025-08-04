function y_pred = get_y_pred(x, g)
%GET_Y_PRED returns the y predicted based on the x predicted
%   Measure = 12-dimensional vector
%   measure = [UWB(pos_n, pos_e, pos_d), Acc (acc_x, acc_y, acc_z), Mag (n, e, d), gyro (wx, wy, wz)]
%   State = 19-dimensional vector
%   [north, east, down, vel_n, vel_e, vel_d, acc_x, acc_y, acc_z, q_w, q_x, q_y, q_z, w_x, w_y, w_z, byas_gx, byas_gy, byas_gz]

% Prediction
    north = x(1); east = x(2); down = x(3);         % pos (NED)
    vel_n = x(4); vel_e = x(5); vel_d = x(6);       % vel (nav frame)
    acc_x = x(7); acc_y = x(8); acc_z = x(9);       % acc (body frame)
    qw = x(10); qx = x(11); qy = x(12); qz = x(13); % quaternion (from nav to body)
    w_x = x(14); w_y = x(15); w_z = x(16);          % omega
    bwx = x(17); bwy = x(18); bwz = x(19);          % byass gyro

    % pos - UWB
    pred_north = north;
    pred_east = east;
    pred_down = down;

    % acc
    pred_accx = acc_x;
    pred_accy = acc_y;
    % pred_accz = acc_z - g;
    pred_accz = acc_z;

    % mag
    pred_mag1 = 0.24*( qw^2 +  qx^2 -  qy^2 -  qz^2)+ 2*0.40*( qw* qy +  qx* qz);
    pred_mag2 = 2*0.40 *( qy* qz -  qw* qx) + 2*0.24*( qw* qz +  qx* qy);
    pred_mag3 = 0.40*( qw^2 -  qx^2 -  qy^2 +  qz^2) +2*0.24 *( qz* qx -  qw* qy);

    % gyro
    pred_gyro_x = w_x + bwx;
    pred_gyro_y = w_y + bwy; 
    pred_gyro_z = w_z + bwz;

    % PITOT:
    % pred_vel_north = vel_n;

    % y_pred = [pred_north, pred_east, pred_down, pred_accx, pred_accy, pred_accz, pred_mag1, pred_mag2, pred_mag3, pred_gyro_x, pred_gyro_y, pred_gyro_z, pred_vel_north]';
    y_pred = [pred_north, pred_east, pred_down, pred_accx, pred_accy, pred_accz, pred_mag1, pred_mag2, pred_mag3, pred_gyro_x, pred_gyro_y, pred_gyro_z]';

end

