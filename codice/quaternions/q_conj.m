function [conj] = q_conj(quat)

% Quaternion conjugate
conjw = quat(1);
conjx = -quat(2);
conjy = -quat(3);
conjz = -quat(4);

conj = [conjw, conjx, conjy, conjz];
end