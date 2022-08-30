%% Read Data
clear;clc;clf;
A = readmatrix('../data/data.csv');
% param = yaml.loadFile("../config/gains.yaml");

ind = 1;
t = (A(2:end,ind)-A(1,ind));        ind=ind+1; % Sample time
pos = A(2:end,ind:ind+2);           ind=ind+3;
quat = A(2:end,ind:ind+3);          ind=ind+4;
wheel_vel = A(2:end,ind:ind+2);     ind=ind+3;
omega = A(2:end,ind:ind+2);         ind=ind+3;
quat_teensy = A(2:end,ind:ind+3);   ind=ind+4;


%%
clf

hfig1 = figure(1);
set(hfig1,'WindowStyle','normal');
tg = uitabgroup(hfig1);

c3 = lines(3);
c4 = lines(4);

tabPlotSingle(tg,t,pos,'Pos')
tabPlotSingle(tg,t,quat,'quat')
tabPlotSingle(tg,t,wheel_vel,'wheel_vel')
tabPlotSingle(tg,t,omega,'omega')
tabPlotSingle(tg,t,quat_teensy,'quat_teensy')


function tabPlot(tg,t,dt,ph,x, x_pred,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'color',c(i,:));
    plot(t+dt*ph,x_pred(:,i),'--','color','k');
end
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end

function tabPlotSingle(tg,t,x,title)
h = uitab(tg, 'Title', title);
ax = axes('Parent', h);
hold on
c = lines(size(x,2));
for i = 1:size(x,2)
    plot(t,x(:,i),'.-','color',c(i,:));
end
% for tau = 0:dt:t(end)
%     line([tau tau],ax.YLim,'color',[0 0 0 .1])
% %     xline(tau,'alpha',.1)
% end
end
