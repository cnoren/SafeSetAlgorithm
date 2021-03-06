function [alpha] = safety_dot_cond(params1, params2, robotstate, compound_x, maxmin)
%safety_dot_cond computes the required condition in (33)
% PARAMS 1 IS STAR!!!!!!!

if maxmin == 0
    alpha = (calc_D(params1, compound_x, robotstate)*calc_D(params2, compound_x, robotstate) + calc_F(params1, compound_x, robotstate)*calc_F(params2, compound_x, robotstate))/((calc_D(params1, compound_x, robotstate))^2 + (calc_F(params1, compound_x, robotstate))^2);
elseif maxmin == 1 
    alpha = -1*(calc_D(params1, compound_x, robotstate)*calc_D(params2, compound_x, robotstate) + calc_F(params1, compound_x, robotstate)*calc_F(params2, compound_x, robotstate))/((calc_D(params1, compound_x, robotstate))^2 + (calc_F(params1, compound_x, robotstate))^2);
else 
    alpha = (calc_D(params1(1:length(params1)/2), compound_x, robotstate)*calc_D(params1((length(params1)/2+1:end)), compound_x, robotstate) + calc_F(params1(1:length(params1)/2), compound_x, robotstate)*calc_F(params1((length(params1)/2+1:end)), compound_x, robotstate))/((calc_D(params1(1:length(params1)/2), compound_x, robotstate))^2 + (calc_F(params1(1:length(params1)/2), compound_x, robotstate))^2);
end
end
