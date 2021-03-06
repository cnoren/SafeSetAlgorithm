function [params] = est_params(q, qdot, qrdot, qrddot, delta_t, gamma, s, barriers, params)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

Y11 = qrddot(1);
Y12 = qrddot(2);
Y21 = 0;
Y22 = qrddot(1) + qrddot(2);
Y13 = (2*qrddot(1) + qrddot(2))*cos(q(2)) - (qdot(2)*qrdot(1) + qdot(1)*qrdot(2) + qdot(2)*qrdot(2))*sin(q(2));
Y23 = qrddot(1)*cos(q(2)) + qdot(1)*qrdot(2)*sin(q(2));

Y = [Y11, Y12, Y13; Y21, Y22, Y23];

adot = -pinv(gamma)*Y'*s

if params(1) + adot(1)*delta_t > barriers(1,2)
    params(1) = barriers(1,2);
elseif params(1) + adot(1)*delta_t < barriers(1,1)
    params(1) = barriers(1,1);
else
    params(1) = params(1) + adot(1)*delta_t;
end

if params(2) + adot(2)*delta_t > barriers(2,2)
    params(2) = barriers(2,2);
elseif params(2) + adot(2)*delta_t < barriers(2,1)
    params(2) = barriers(2,1);
else
    params(2) = params(2) + adot(2)*delta_t;
end

if params(3) + adot(3)*delta_t > barriers(3,2)
    params(3) = barriers(3,2);
elseif params(3) + adot(3)*delta_t < barriers(3,1)
    params(3) = barriers(3,1);
else
    params(3) = params(3) + adot(3)*delta_t;
end
end

