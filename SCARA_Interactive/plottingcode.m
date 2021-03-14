%Plotting stuff

% Get the distance
for i = 1:1:length(robot.profile)
    rmin_list(i) = robot.profile{i}.rmin;
end

figure
hold on
grid on
p1 = plot(1:1:length(robot.profile), rmin_list(1:length(robot.profile)), 'linewidth', 2);
p2 = plot(1:1:length(robot.profile), 0.15*ones(1000,1), 'linewidth', 2);
lgd = legend([p1, p2], 'Minimum Distance to Manipulator', 'Safe Distance', 'interpreter','latex');
lgd.FontSize = 12;
xlabel('Time Step', 'interpreter','latex', 'FontSize', 12)
ylabel('Distance [m]', 'interpreter','latex', 'FontSize', 12)
hold off
close


figure
hold on
grid on
p1 = plot(1:1:length(robot.x)-100, robot.x(3,1:length(robot.x)-100), 'linewidth', 2);
p2 = plot(1:1:length(robot.x)-100, robot.x(4,1:length(robot.x)-100), 'linewidth', 2);
lgd = legend([p1, p2], 'Velocity of Joint 1', 'Velocity of Joint 2', 'interpreter','latex');
lgd.FontSize = 12;
xlabel('Time Step', 'interpreter','latex', 'FontSize', 12)
ylabel('Angular Velocity [rad/s]', 'interpreter','latex', 'FontSize', 12)
hold off
close

figure
hold on
grid on
p1 = plot(1:1:length(robot.u)-100, robot.u(1,1:length(robot.u)-100), 'linewidth', 2);
p2 = plot(1:1:length(robot.u)-100, robot.u(2,1:length(robot.u)-100), 'linewidth', 2);
lgd = legend([p1, p2], 'Torque, Joint 1', 'Torque, Joint 2', 'interpreter','latex');
lgd.FontSize = 12;
xlabel('Time Step', 'interpreter','latex', 'FontSize', 12)
ylabel('Torque [Nm]', 'interpreter','latex', 'FontSize', 12)
hold off
close