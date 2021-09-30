% % z=[];
% % x=[];
% % dotz=[];
% % dotx=[];
% % for t=0:0.001:0.3
% %     if t<0.25/4
% %         x(end+1)  = -4*0.1*t*t/0.25 + 0.1*t  -0.1;
% %         dotx(end+1)= -8 * 0.1 * t / 0.25 + 0.1;
% %     elseif  t>0.25/4 &&t<3/4*0.25
% %         dotx(end+1)=(-4 * 0.25 * 0.1 - 16 * 0.1 + 16 * -0.1) * 3 * t * t / (0.25 * 0.25 * 0.25) + (7 * 0.25 * 0.1 + 24 * 0.1 - 24 * -0.1) * 2 * t / (0.25 * 0.25) + (-15 * 0.25 * 0.1 - 36 * 0.1 + 36 * -0.1) / (4 * 0.25);
% %         x(end+1)=( -4*0.25*0.1 - 16*0.1 + 16*-0.1)*t*t*t/(0.25*0.25*0.25) + (  7*0.25*0.1 + 24*0.1 - 24*-0.1)*t*t/(0.25*0.25) + (-15*0.25*0.1 - 36*0.1 + 36*-0.1)*t/(4*0.25) + (  9*0.25*0.1 + 16*0.1)/16;
% %     else
% %         x(end+1)=0.1;
% %         dotx(end+1)=0;
% %     end
% %       
% % %     if t<0.25*2/5
% % %         z(end+1)=16*(-0.55 +0.4)*t^3/(0.25^3) + 12*(-0.4 +0.55)*t^2/(0.25^2)-0.55;
% % %         dotz(end+1) = 16 * (-0.55 - -0.4) * 3 * t * t / (0.25 * 0.25 * 0.25) + 12 * (-0.4 - -0.55) * 2 * t / (0.25 * 0.25);
% % %     elseif  (t >= 0.25*2/5) && (t < 2.5 * 0.25 / 4.0)
% % %         dotz(end+1) =1*sin(2*pi/0.6*t+0.9) +0.31       %4 * (-0.55 - -0.4) * 2 * t / (0.25 * 0.25) - 4 * (-0.55 - -0.4) / 0.25;
% % %         z(end+1)=4*(-0.55 +0.4).*t.^2/(0.25^2) - 4*(-0.55 +0.4).*t/0.25-0.55;
% % %     else
% % %         z(end+1)=4*(-0.55 +0.4).*t.^2/(0.25^2) - 4*(-0.55 +0.4).*t/0.25-0.55;
% % %         dotz(end+1) =1*sin(2*pi/0.6*t+0.9)+0.31
% %         
% %     end
% % end
% %example2.8
% clc
% clear
% %轨迹定义条件
% t_array=[0,0.25/2,0.25*3/5,0.25*4/5,0.25];
% q_array=[-0.6,-0.4,-0.42,-0.6,1.4];
% v_array=[0,0.4,0.4,0,0];
% %计算轨迹
% %初始位置
% t=t_array(1);
% q=q_array(1);
% v=v_array(1);
% v_array2=v_array;
% 
% for k=1:length(t_array)-1
%     %按照式（1-23）式确定中间点的速度值
%     if(k>1)
%         dk1=(q_array(k)-q_array(k-1))/(t_array(k)-t_array(k-1));
%         dk2=(q_array(k+1)-q_array(k))/(t_array(k+1)-t_array(k));
%         if((dk2>=0 && dk1>=0) || (dk2<=0 && dk1<=0))
%             v_array2(k)=1.0/2.0*(dk1+dk2);
%         else
%             v_array2(k)=0;
%         end  
%     end
% end
% 
% %计算各段轨迹
% for k=1:length(t_array)-1
%     %计算各段多项式的系数
%     h(k)=q_array(k+1)-q_array(k);
%     T(k)=t_array(k+1)-t_array(k);
%     a0(k)=q_array(k);
%     a1(k)=v_array2(k);
%     a2(k)=(3*h(k)-(2*v_array2(k)+v_array2(k+1))*T(k))/(T(k)*T(k));
%     a3(k)=(-2*h(k)+(v_array2(k)+v_array2(k+1))*T(k))/(T(k)*T(k)*T(k));
% 
%     %生成各段轨迹密化的数据点
%     %局部时间坐标
%     tau=t_array(k):T(k)/100:t_array(k+1);
%     %全局时间坐标，由局部时间坐标组成
%     t=[t,tau(2:end)];
%     %局部位置坐标
%     qk=a0(k)+a1(k)*power(tau-tau(k),1)+a2(k)*power(tau-tau(k),2)+a3(k)*power(tau-tau(k),3);
%     %全局位置坐标
%     q=[q,qk(2:end)];
%     %速度
%     vk=a1(k)+2*a2(k)*power(tau-tau(k),1)+3*a3(k)*power(tau-tau(k),2);
%     v=[v,vk(2:end)];
%     %加速度
%     acck=2*a2(k)+6*a3(k)*power(tau-tau(k),1);
%     if(k==1)
%         acc=2*a2(k);
%     end
%     acc=[acc,acck(2:end)];
% end
% %绘图
% subplot(3,1,1);
% h2=plot(t,q,'--r');
% legend(h2,'第二种方式')
% hold on;
% plot(t_array,q_array,'^r');
% %axis([0,10,-5,45]);
% ylabel('position')
% grid on;
% subplot(3,1,2);
% plot(t_array,v_array2,'^b');
% hold on;
% plot(t,v,'--b');
% %axis([0,10,-20,15]);
% ylabel('velocity')
% grid on;
% subplot(3,1,3);
% plot(t,acc,'--g');
% %axis([0,10,-45,45]);
% ylabel('acceleration')
% grid on;
% % ————————————————
% % 版权声明：本文为CSDN博主「Brian2018」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
% % 原文链接：https://blog.csdn.net/libing403/article/details/78698322
% 
% z=[]
% syms xf xh yf zf zh yh zh
% Q=[[1 0 0 1 0 0];
%     [0 0 1 0 0 1];
%     [0 -zf yf 0 -zh yh];
%     [zf 0 -xf zh 0 -xh];
%     [-yf xf 0 -yh xh 0];
%     [0 1 0 0 -1 0]];
% q=inv(Q);
% t=inv(Q);
% [x,z]=size(q);
% for i =1:x
% for j = 1:z
%     t(i,j)=simplify(q(i,j))
% end
% end
A2=1.02;
D4=2.57;
data=[8.3,9.1,8.6,10.6,9.0,8.8,8.9,9.9,10.6,9.2,8.9,9,9.7,8.6,10.7,8.7,9.9,10.2,8.4,8.4;
    8.9 9.8 8 8.6 8.5 9.8 8.7 8.7 11.9 9.0 10.8 7.9 8.5 9.8 10.7 9.6 9 8.5 9.7 10.2 
    9.4 8.5 9.2 9 9.3 8.3 11 9 8.2 9.4 8.7 7.9 9.6 9.2 9.3 9.4 8.8 9.4 9 10];
data=data';
x_=[] ;
R=[];
for i=1:20
    x_(end+1)=roundn(mean(data(i,:)),-2);
    R(end+1)=max(data(i,:))-min(data(i,:));
end
X__= roundn(mean(x_),-2);
R_= roundn(mean(R),-2)
t=1:20;
hold on;
% plot([1,20],[roundn(mean(x_),-2),roundn(mean(x_),-2)])
% plot([1,20],[roundn(mean(x_),-2)+A2*R_,roundn(mean(x_),-2)+A2*R_])
% plot([1,20],[roundn(mean(x_),-2)-A2*R_,roundn(mean(x_),-2)-A2*R_])
% plot(t,x_)

plot([1,20],[R_,R_])
plot([1,20],[R_*D4,R_*D4])
plot([1,20],[R_*0,R_*0])
plot(t,R)
