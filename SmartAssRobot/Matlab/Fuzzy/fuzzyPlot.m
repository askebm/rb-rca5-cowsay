%%
m = readmatrix("ActivationRate.txt");
height = 250;

figure('Name','float_me','Position',[0 0 600*0.6 height])
plot3(m(:,1),m(:,2),m(:,3),'.')
xlabel('obstacle')
ylabel('distance')
zlabel('speed')
view(135,45)

Plot2LaTeX(gcf,'ObsDisSpeed')

figure('Name','float_me','Position',[0 0 600*0.6 height])
plot3(m(:,1),m(:,2),m(:,4),'.')
xlabel('obstacle')
ylabel('distance')
zlabel('steer')
view(45,45)

Plot2LaTeX(gcf,'ObsDisSteer')


m = readmatrix("ActiMarbCathcer.txt")
figure('Name','float_me','Position',[0 0 600*0.6 height])
plot3(m(:,3),m(:,2),m(:,1),'.')
xlabel('steer')
ylabel('speed')
zlabel('errorYaw')
view(90,0)
Plot2LaTeX(gcf,'SpeErr')

figure('Name','float_me','Position',[0 0 600*0.6 height])
plot3(m(:,3),m(:,2),m(:,1),'.')
xlabel('steer')
ylabel('speed')
zlabel('errorYaw')
view(0,0)
Plot2LaTeX(gcf,'SteErr')

figure('Name','float_me','Position',[0 0 600*0.6 height])
plot3(m(:,3),m(:,2),m(:,1),'.')
xlabel('steer')
ylabel('speed')
zlabel('errorYaw')
Plot2LaTeX(gcf,'SteSpeErr')
%%
