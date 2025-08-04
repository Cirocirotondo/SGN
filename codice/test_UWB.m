% function UWB_pos_ned = test_UWB(pos_ned)

posizione_vera = [200, 500 , 0]';

P_base1_ned = [0; 0; 0];        % North [m], East [m], Down [m]
P_base2_ned = [1000; 12; 1000];
P_base3_ned = [1500; 20; -400];
P_base4_ned = [70; 1400; 20];

distanze = [ norm(posizione_vera-P_base1_ned); 
      norm(posizione_vera-P_base2_ned);
      norm(posizione_vera-P_base3_ned);
      norm(posizione_vera-P_base4_ned) ]
    
d1 = distanze(1);
d2 = distanze(2);
d3 = distanze(3);
d4 = distanze(4);
    
A = [(P_base1_ned-P_base4_ned)';
     (P_base2_ned-P_base4_ned)';
     (P_base3_ned-P_base4_ned)' ];

C = [d4^2-d1^2-sum(P_base4_ned.^2)+sum(P_base1_ned.^2);
     d4^2-d2^2-sum(P_base4_ned.^2)+sum(P_base2_ned.^2);
     d4^2-d3^2-sum(P_base4_ned.^2)+sum(P_base3_ned.^2)];
    
posizione_UWB = 1/2 * inv(A' * A) * A' * C
posizione_UWB = 0.5 * pinv(A) * C

% end


%%
% Estimate the 3D position in NED coordinates given distances to 4 known points

% Stack antenna positions
A = [P_base1_ned';  % [N E D]
     P_base2_ned';
     P_base3_ned';
     P_base4_ned'];


% Extract reference (first) base
x1 = A(1,1); y1 = A(1,2); z1 = A(1,3);
d1 = distanze(1);

% Set up linear system
M = zeros(3,3);
b = zeros(3,1);

for i = 2:4
    xi = A(i,1); yi = A(i,2); zi = A(i,3);
    di = distanze(i);
    
    M(i-1, :) = 2 * [x1 - xi, y1 - yi, z1 - zi];
    b(i-1) = ...
        x1^2 - xi^2 + ...
        y1^2 - yi^2 + ...
        z1^2 - zi^2 + ...
        di^2 - d1^2;
end

% Solve for position
pos_est_ned = (M \ b)'  % Transpose for 1x3 output
