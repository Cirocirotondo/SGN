function Vect_b = navi2body(Q_nb,Vect_n)
% Rotation from naviation frame to body frame with quaternion

% Vector as pure quaternion
Quat_n = [0, Vect_n(1), Vect_n(2), Vect_n(3)];

% Rotation in body frame -> V_b = Q_NB * V_nav * conj(Q_NB)
Q_MULT_P = q_multi(Quat_n, q_conj(Q_nb));
quat_b = q_multi(Q_nb, Q_MULT_P);

% From pure quaternion to vector
Vect_b = [quat_b(2), quat_b(3), quat_b(4)];
end