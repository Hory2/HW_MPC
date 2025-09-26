close all
w=1.3
figure
beg=5500;
plot(uk21a.uk21a.time(1:beg),uk21a.uk21a.signals(1).values((1:beg),[1]),LineWidth=1,LineStyle="--")
hold on
plot(uk21a.uk21a.time(1:beg),uk21a.uk21a.signals(1).values((1:beg),[3]),LineWidth=w)
grid on
plot(uk21b.uk21a.time(1:beg),uk21b.uk21a.signals(1).values((1:beg),[3]),LineWidth=w)
xlabel("t[s]")
ylabel("I[A]")

legend('i_{ref}','i_{MPC}','i_{hyster}')

saveas(gcf, ['HysterVZmpc.png']);
%%

 figure
beg=5000;
plot(uk22a.uk21a.time(1:beg),uk21a.uk21a.signals(1).values((1:beg),[1]),LineWidth=1,LineStyle="--")
hold on
plot(uk22a.uk21a.time(1:beg),uk22a.uk21a.signals(1).values((1:beg),[3]),LineWidth=w)
grid on
plot(uk22b.uk21a.time(1:beg),uk22b.uk21a.signals(1).values((1:beg),[3]),LineWidth=w)
xlabel("t[s]")
ylabel("I[A]")

legend('i_{ref}','\lambda_u = 20e-3 ','\lambda_u = 25e-3')

saveas(gcf, ['lambdy.png']);

%%
 
 close all
 figure
beg=4000;
yyaxis left
plot(uk25.uk21a.time(1:beg),uk25.uk21a.signals(1).values((1:beg),[1]),LineWidth=1,LineStyle="--",Color='m')
hold on
plot(uk25.uk21a.time(1:beg),uk25.uk21a.signals(1).values((1:beg),[3]),LineWidth=w,LineStyle="-")
grid on
yyaxis right
plot(uk25.uk21a.time(1:beg),uk25.uk21a.signals(2).values((1:beg),:),LineWidth=0.6,Color='k')
ylim([-3,3])
% saveas(gcf, ['uk25a.png']);

 figure
beg=4000;
yyaxis left
plot(uk25b.uk21a.time(1:beg),uk25b.uk21a.signals(1).values((1:beg),[1]),LineWidth=1,LineStyle="--",Color='m')
hold on
plot(uk25b.uk21a.time(1:beg),uk25b.uk21a.signals(1).values((1:beg),[3]),LineWidth=w,LineStyle="-")
grid on
yyaxis right
plot(uk25.uk21a.time(1:beg),uk25b.uk21a.signals(2).values((1:beg),:),LineWidth=0.6,Color='k')
ylim([-3,3])
% saveas(gcf, ['uk25b.png']);
%%
close all
w=1.3
figure
beg=6500;
plot(uk27.uk21a.time(1:beg),uk27.uk21a.signals(1).values((1:beg),[1]),LineWidth=1,LineStyle="--")
hold on
plot(uk27.uk21a.time(1:beg),uk27.uk21a.signals(1).values((1:beg),[3]),LineWidth=w)
grid on
xlabel("t[s]")
ylabel("I[A]")

legend('i_{ref}','i_{out}')

saveas(gcf, ['LL.png']);