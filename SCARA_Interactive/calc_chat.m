function [chat] = calc_chat(pos, velo, param)
%calculating chat
alpha=param(1);
beta=param(2);
delta=param(3);

h = delta*sin(pos(2));

chat = [-h*velo(2), -h*(velo(1)+velo(2)); h*velo(1), 0];


end

