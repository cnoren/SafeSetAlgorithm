function [alpha_condition, M_safe1, M_safe2, flag] = evolutionary(centerpoint, param_LB, param_UB, jacobian, distance, robotstate, range, grids)
% This is an evolutionary method with computes the following maximin
% problem:

EV_ALGO_FLAG = 0;
flag = 0;
Terr = 1*10^(-3);
err = 1*10^12;
options = optimoptions(@fmincon,'Display','off');
options2 = optimoptions(@fminimax,'Display','off');
compound_x = jacobian*distance;
% param1 = g*, param2 = g
% IL1 = computeValInRange(range.IL1(1), range.IL1(2), rng(s, 'twister'), 0);
% IL2 = computeValInRange(range.IL2(1), range.IL2(2), rng(s, 'twister'), 0);
% lL1 = computeValInRange(range.lL1(1), range.lL1(2), rng(s, 'twister'), 0);
% lL2 = computeValInRange(range.lL2(1), range.lL2(2), rng(s, 'twister'), 0);
% mL1 = computeValInRange(range.mL1(1), range.mL1(2), rng(s, 'twister'), 0);
% mL2 = computeValInRange(range.mL2(1), range.mL2(2), rng(s, 'twister'), 0);
% mM2 = computeValInRange(range.mM2(1), range.mM2(2), rng(s, 'twister'), 0);
% mEE = computeValInRange(range.mEE(1), range.mEE(2), rng(s, 'twister'), 0);
% TC1 = computeValInRange(range.TC1(1), range.TC1(2), rng(s, 'twister'), 0);
% TC2 = computeValInRange(range.TC2(1), range.TC2(2), rng(s, 'twister'), 0);
% param2 = [IL1; IL2; lL1; lL2; mL1; mL2; mM2; mEE; TC1; TC2;];
% [n_param2, fval] = fmincon(@(x)safety_dot_cond(param1, x, robotstate, compound_x,0), param2, [], [], [], [], param_LB, param_UB);
% n_param2 - param_LB
param1 = [centerpoint]; param2 = param_UB;
param_total = [param1; param2];
if EV_ALGO_FLAG
    while err > Terr
        % MAXIMIN PROBLEM!
        %n_param2 = fmincon(@(x)safety_dot_cond(param1, x, robotstate, compound_x,0), param2, [], [], [], [], param_LB, param_UB);
        %n_param2 = fmincon(@(x)safety_dot_cond(param1, x, robotstate,
        %compound_x,0), param2, [], [], [], [], param_LB, param_UB, [],
        %options); % NO OUTPUT
        % Solve fmincon for max on m(s)
        %n_param1 = fmincon(@(x)safety_dot_cond(x, param2, robotstate, compound_x,1), param1, [], [], [], [], param_LB, param_UB);
        %n_param1 = fmincon(@(x)safety_dot_cond(x, param2, robotstate,
        %compound_x,1), param1, [], [], [], [], param_LB, param_UB, [], options); % NO OUTPUT
        % Compare the old reference with new solution --> update new error
        %param_total_new = [n_param1; n_param2];
        %err = norm((param_total - param_total_new));
        %param_total = param_total_new;
        %param1 = n_param1; param2 = n_param2;

        % CHECKING THE MIN CASE
        %[n_param, fval] = fmincon(@(x)safety_dot_cond(x, param2, robotstate, compound_x,2), param_total, [], [], [], [], [param_LB; param_LB], [param_UB; param_UB]);
        [n_param, fval] = fmincon(@(x)safety_dot_cond(x, param2, robotstate, compound_x,2), param_total, [], [], [], [], [param_LB; param_LB], [param_UB; param_UB], [], options);
        err = norm((param_total - n_param));
        param_total = n_param;
    end

    if fval<0
        disp('FVAL IS LESS THAN 0!!!!!!')
        flag = 1
    end
    % MAXIMIN CASE: calculate alpha, beta, epsilon
    % alpha_condition = safety_dot_cond(param1, param2, robotstate, compound_x, 0);
    % alpha = calcParam_alpha(param1);
    % beta = calcParam_beta(param1);
    % epsilon = calcParam_epsilon(param1);
    % 
    % M_safe1 = [alpha + 2*epsilon*cos(robotstate(2)), beta + epsilon*cos(robotstate(2)); beta + epsilon*cos(robotstate(2)), beta];
    % M_safe2 = [];

    % MIN CASE: calculate alpha, beta, epsilon
    alpha_condition = safety_dot_cond(param_total(1:length(param_total)/2), param_total(length(param_total)/2+1:end), robotstate, compound_x, 0);
    alpha = calcParam_alpha(param_total(1:length(param_total)/2));
    beta = calcParam_beta(param_total(1:length(param_total)/2));
    epsilon = calcParam_epsilon(param_total(1:length(param_total)/2));

    M_safe1 = [alpha + 2*epsilon*cos(robotstate(2)), beta + epsilon*cos(robotstate(2)); beta + epsilon*cos(robotstate(2)), beta];

    alpha = calcParam_alpha(param_total(length(param_total)/2+1:end));
    beta = calcParam_beta(param_total(length(param_total)/2+1:end));
    epsilon = calcParam_epsilon(param_total(length(param_total)/2+1:end));

    M_safe2 = [alpha + 2*epsilon*cos(robotstate(2)), beta + epsilon*cos(robotstate(2)); beta + epsilon*cos(robotstate(2)), beta];
else
    abe_funclist = @(x)safety_dot_cond_discret(grids.reshapedalphaGrid, grids.reshapedbetaGrid, grids.reshapedepsilonGrid, x, robotstate, compound_x);
    %[params_min, fval] = fminimax(abe_funclist, param1, [], [], [],[], param_LB, param_UB, []);
    [params_min, fval] = fminimax(abe_funclist, param1, [], [], [], [], param_LB, param_UB, [], options2);
    fval  = min(fval);
    if fval<0
        disp('FVAL IS LESS THAN 0!!!!!!')
        flag = 1
    end

    alpha_condition = fval;
    alpha = calcParam_alpha(params_min);
    beta = calcParam_beta(params_min);
    epsilon = calcParam_epsilon(params_min);
    
    M_safe1 = [alpha + 2*epsilon*cos(robotstate(2)), beta + epsilon*cos(robotstate(2)); beta + epsilon*cos(robotstate(2)), beta];    
    M_safe2 = [];

end
end


