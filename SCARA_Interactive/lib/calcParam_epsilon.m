function [epsilon] = calcParam_epsilon(parameter_vals)
%calcParam_epsilon computes the epsilon parameter
        %robot.est_I    =[IL1 IL2]; %Moments of inertia (kg.m^2)
        %robot.est_l    =[lL1 lL2]; %Length of the links (m)
        %robot.est_m    =[mL1 mL2]; %Mass of the links (Kg)
        %robot.est_M    =[mM2 mEE]; %Mass of the second motor and the end-effector
        %robot.est_Kt   =[TC1 TC2]; %Torque constant of the motors (N.m/V)
        %robot.est_DH   =[0 0 robot.l(1) 0;pi/4 0 robot.l(2) 0]; %theta,d,a,alpha
        % robot.centerpoint = [IL1; IL2; lL1; llL2; mL1; mL2; mM2; mEE; TC1; TC2];
        % There is an implicit assumption here that the center of mass of
        % each of the links is located at the length-centroid of the link
        % We don't use the extra masses that are currently written out

epsilon = parameter_vals(6)*parameter_vals(3)*parameter_vals(4)/2;
end