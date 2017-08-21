clear all;
x=1:1:220;  
%y=4*rand(1,220);
%save('y.mat','y');
%y1=4*randn(1,220);
%save('y1.mat','y');
load('2-y1.mat');
load('2-MDnet-y1.mat');
load('2-CMOT-y1.mat');
figure;
hold on;
%�����߿�(��λpoints)����߿���ɫ������ɫ�����С
plot(x,y,'-.rs','LineWidth',1,'MarkerFaceColor','r','MarkerSize',2);
plot(x,y1,'-.bv','LineWidth',1,'MarkerFaceColor','b','MarkerSize',2);
plot(x,y2,'-.m*','LineWidth',1,'MarkerFaceColor','m','MarkerSize',2);
axis([1 230 0 50]);
ylabel('�������/px');
xlabel('֡��');
set(gca,'XTick',[0:20:220]);%����Ҫ��ʾ����̶�
set(gca,'YTick',[0:5:50]);%������ӱ�ǩ 
legend('Ours','CMOT','MDP');