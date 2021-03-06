function [D] = calc_D(params_small, compound_x, robotstate)
%calc_D -- calculates the d term

D = (calcParam_beta(params_small)*compound_x(1) + calcParam_epsilon(params_small)*compound_x(1)*cos(robotstate(2)) - calcParam_alpha(params_small)*compound_x(2) - 2*calcParam_epsilon(params_small)*cos(robotstate(2))*compound_x(2));
end

