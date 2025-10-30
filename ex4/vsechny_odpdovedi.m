why

%% prumer pro ukol 1.4
awg1=mean(out1.switching.signals.values)

%% 1.5 porovnani prvotnich odhadu
% figure
% plot(out2.iteraions.time,out2.iteraions.signals.values)
% hold on
% plot(out1.iteraions.time,out1.iteraions.signals.values)

figure
hold on
histogram(out1.iteraions.signals.values)
histogram(out2.iteraions.signals.values)
xlabel('number of iteration needed')
ylabel('frequency')
legend('zeros', 'shifted previous switching sequence')
saveas(gcf, ['histogram.png']);

%% 1.6 moment
figure
subplot(2,1,1)
plot(out3.torque_scope.time,out3.torque_scope.signals(1).values(:,1))
hold on
plot(out3.torque_scope.time,out3.torque_scope.signals(1).values(:,2))
grid on
xlabel('t[s]')
ylabel('T[p.u]')
legend('T_{ref}','T')

subplot(2,1,2)
plot(out3.scope_1.time,out3.scope_1.signals(1).values(:,1))
hold on
plot(out3.scope_1.time,out3.scope_1.signals(1).values(:,2))
hold on
xlabel('t[s]')
ylabel('i[p.u]')
legend('i_{\alpha}','i_{\beta}')
saveas(gcf, ['moment.png']);

%% 2.2  nu=3 -out4
awg2=mean(out4.switching.signals.values)
figure
hold on
awqIt=mean(out4.iteraions.signals.values)
histogram(out4.iteraions.signals.values)
histogram(out2.iteraions.signals.values)
xlabel('number of iteration needed')
ylabel('frequency')
legend('N_u=2', 'N_u=3')
saveas(gcf, ['histo2.png']);

%% 1.4 babai


awg2=mean(out4.switching.signals.values)
figure
hold on
histogram(out5.iteraions.signals.values)

histogram(out4.iteraions.signals.values)
xlabel('number of iteration needed')
ylabel('frequency')
legend( 'babai','orig')
 saveas(gcf, ['histo3.png']);

