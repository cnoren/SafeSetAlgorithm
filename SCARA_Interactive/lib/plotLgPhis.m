function [outputArg1,outputArg2] = plotLgPhis(m, mhat, msafe, msafeLP, msafeUP, jacobian, dist)
%helper function to visualize the lie derivative direction

LD_m = m^(-1) * jacobian * dist;
LD_mhat = mhat^(-1) * jacobian * dist;
LD_msafe = msafe^(-1) * jacobian * dist;
LD_msafeLP = msafeLP^(-1) * jacobian * dist;
LD_msafeUP = msafeUP^(-1) * jacobian * dist;

fig2 = figure
grid on 
hold on
plot([0, LD_m(1)], [0, LD_m(2)], 'r--O')
plot([0, LD_mhat(1)], [0, LD_mhat(2)], 'b--O')
plot([0, LD_msafe(1)], [0, LD_msafe(2)], 'k--O')
plot([0, LD_msafeLP(1)], [0, LD_msafeLP(2)], 'c--O')
plot([0, LD_msafeUP(1)], [0, LD_msafeUP(2)], 'g--O')
legend('True', 'Estimate', 'g*', 'g')
close(fig2)
end

