function [F] = calc_F(params_small, compound_x, robotstate)
%calc_F -- calculates the F term

F = (calcParam_beta(params_small)*compound_x(2) + calcParam_epsilon(params_small)*cos(robotstate(2))*compound_x(2) - calcParam_beta(params_small)*compound_x(1));
end
