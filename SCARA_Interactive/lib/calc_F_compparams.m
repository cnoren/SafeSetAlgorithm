function [F] = calc_F_compparams(beta, epsilon, compound_x, robotstate)
%calc_F -- calculates the F term

F = (beta.*compound_x(2) + epsilon.*cos(robotstate(2)).*compound_x(2) - beta.*compound_x(1));
end