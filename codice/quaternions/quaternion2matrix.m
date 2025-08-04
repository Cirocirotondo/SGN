function Rot_matrix = quaternion2matrix(quaternion)

% conversion quaternion -> Rotation matrix

q0 = quaternion(1);
q1 = quaternion(2);
q2 = quaternion(3);
q3 = quaternion(4);
Rot_matrix = [q0^2+q1^2-q2^2-q3^2,  2*(q1*q2-q0*q3),        2*(q1*q3+q0*q2);
              2*(q1*q2+q0*q3),      q0^2-q1^2+q2^2-q3^2,    2*(q2*q3-q0*q1);
              2*(q1*q3-q0*q2),      2*(q2*q3+q0*q1),        q0^2-q1^2-q2^2+q3^2];
end 