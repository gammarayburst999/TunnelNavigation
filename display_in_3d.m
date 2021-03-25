sim('test_01_3d_veh_piby567_2018')

robo_Position= [x_01.signals(1).values y_01.signals(1).values z_01.signals(1).values ];
Control_signal = [Control.signals(1).values Control.time];
%% Plot figures
figure (1)
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on

plot3(robo_Position(4,1),robo_Position(4,2),robo_Position(4,3),'.','Color','k');
h = animatedline('LineWidth',1.2);

for i = 1:length(robo_Position)
    addpoints(h,robo_Position(i,1),robo_Position(i,2),robo_Position(i,3));
    drawnow
    %pause(0.1)
end
xlabel('Position in Cartesian coordinate [m]')
ylabel('Position in Cartesian coordinate [m]')
zlabel('Position in Cartesian coordinate [m]')
title('Simple tunnel navigation along center-line')
set(get(get(h1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
legend('Start position','Vehicle trajectory')
 
%% Control
figure(3)
plot(Control_signal(:,4), Control_signal(:,1),'Linewidth',1)
% hold on
% plot(Control_signal(:,4), Control_signal(:,2),'Linewidth',1)
% plot(Control_signal(:,4), Control_signal(:,3),'Linewidth',1)
xlabel('Time [s]')
ylabel('Control signal [-]')
legend('$\dot{\psi}(t)$','interpreter','latex')
title('Tunnel navigation along center-line - Control signal')
xlim([0 Control_signal(end,4)])
ylim([-1.2 1.2])
grid on