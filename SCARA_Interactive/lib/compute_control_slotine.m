function [U] = compute_control_slotine(qrddot, qrdot, s, H, C, KD)
%compute_control_slotine: computes the CLAW based on Slotines seminal 1986
%paper sans the use of the gravitational component (we assume in plane)
% INPUTS:
%   H - The inertial matrix
%   C - The Coriolis matrix
%   qrddot - This is a combination variable that is computed for the
%   sliding surface at the acceleration level [rad/s^2]
% OUTPUTS:
%   U - THIS IS A 2 \times 1 vector of the Torque output: [motor1, motor2]
%   in [Newton*Meters]

U = H*qrddot + C*qrdot - KD*s;
end

