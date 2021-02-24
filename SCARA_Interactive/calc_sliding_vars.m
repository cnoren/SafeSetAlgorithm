function [qrdot, qrddot, s] = calc_sliding_vars(velo, goal, ee_pos, ee_vel, J, H, lambda)
%calc_sliding_vars: compute the values needed in the sliding variable
%system for use in the control design
% INPUTS: 
%     J - The system geometric Jacobian
%     H - The system Hessian
%     lambda - Control system parameter
% OUTPUTS:
%     qrdot - Sliding Variable qrdot (velocity level)
%     qrddot - Sliding Variable qrddot (acceleration level)
%     s - Sliding Variable

qrdot = pinv(J)*(zeros(2,1) + lambda*(goal - ee_pos));
qrddot = pinv(J)*(zeros(2,1) + lambda*(zeros(2,1) - ee_vel)) - H*qrdot;
s = pinv(J)*(J*velo - zeros(2,1) + lambda*(ee_pos - goal));


end

