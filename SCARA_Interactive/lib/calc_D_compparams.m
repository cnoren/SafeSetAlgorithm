function [D] = calc_D_compparams(alpha, beta, epsilon, compound_x, robotstate)
%calc_D -- calculates the d term

D = (beta.*compound_x(1) + epsilon.*compound_x(1).*cos(robotstate(2)) - alpha.*compound_x(2) - 2.*epsilon.*cos(robotstate(2)).*compound_x(2));
end