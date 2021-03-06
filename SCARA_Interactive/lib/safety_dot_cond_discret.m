function [alpha] = safety_dot_cond_discret(alpha_p, beta_p, ep_p, params2, robotstate, compound_x)
%safety_dot_cond computes the required condition in (33)
% PARAMS 1 IS STAR!!!!!!!
T1 = calc_D_compparams(alpha_p, beta_p, ep_p, compound_x, robotstate);
T2 = calc_D(params2, compound_x, robotstate);
T3 = calc_F_compparams(beta_p, ep_p, compound_x, robotstate);
T4 = calc_F(params2, compound_x, robotstate);

alpha = (T1.*T2 + T3.*T4)./((T1).^2 + (T2).^2);
end

