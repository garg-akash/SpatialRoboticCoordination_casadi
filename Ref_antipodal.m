%File to generate reference trajectory for antipodal configuration
s_1 = [10,0];
g_1 = [0,10];

s_2 = [0,0];
g_2 = [10,10];

s_3 = [0,10];
g_3 = [10,0];

s_4 = [10,10];
g_4 = [0,0];

total_t = 40;
%th = pi:pi/39:2*pi;
del_t = 1;
avg_vel_1 = (g_1 - s_1)/(total_t*del_t);
avg_vel_2 = (g_2 - s_2)/(total_t*del_t);
avg_vel_3 = (g_3 - s_3)/(total_t*del_t);
avg_vel_4 = (g_4 - s_4)/(total_t*del_t);

%%footprint of obstacle/unit 1
hinit_1 = 3*pi/4; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;

%%footprint of obstacle/unit 3
hinit_3 = -pi/4; % initial heading for unit 3
b_3 = 0.75;
l_3 = 1.5;

%%footprint of obstacle/unit 4
hinit_4 = 5*pi/4; % initial heading for unit 4
b_4 = 0.75;
l_4 = 1.5;

%%footprint of robot/unit 2
hinit_2 = pi/4; % initial heading for unit 2
b_2 = 0.75;
l_2 = 1.5; 

Xunit_1=[];
Yunit_1=[];
Hfinal_1=hinit_1*ones(total_t,1);

Xunit_3 = [];
Yunit_3 = [];
Hfinal_3 = hinit_3*ones(total_t,1);

Xunit_4=[];
Yunit_4=[];
Hfinal_4=hinit_4*ones(total_t,1);

Xunit_2=[];
Yunit_2=[];
Hfinal_2=hinit_2*ones(total_t,1);
Vxunit_2=[];
Vyunit_2=[];

xnow_1 = s_1(1);
ynow_1 = s_1(2);
xnow_3 = s_3(1);
ynow_3 = s_3(2);
xnow_4 = s_4(1);
ynow_4 = s_4(2);

xnow_2 = s_2(1);
ynow_2 = s_2(2);

for i=1:total_t
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    
    xunit_3 = xnow_3;
    yunit_3 = ynow_3;
    Xunit_3 = [Xunit_3,xunit_3];
    Yunit_3 = [Yunit_3,yunit_3];
    
    xunit_4 = xnow_4;
    yunit_4 = ynow_4;
    Xunit_4 = [Xunit_4,xunit_4];
    Yunit_4 = [Yunit_4,yunit_4];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    
    xdel_1 = avg_vel_1(1)*del_t;
    ydel_1 = avg_vel_1(2)*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_3 = avg_vel_3(1)*del_t;
    ydel_3 = avg_vel_3(2)*del_t;
    xnow_3 = xdel_3 + xunit_3;
    ynow_3 = ydel_3 + yunit_3;
    
    xdel_4 = avg_vel_4(1)*del_t;
    ydel_4 = avg_vel_4(2)*del_t;
    xnow_4 = xdel_4 + xunit_4;
    ynow_4 = ydel_4 + yunit_4;
    
    xdel_2 = avg_vel_2(1)*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = avg_vel_2(2)*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;avg_vel_2(1)];
    Vyunit_2 = [Vyunit_2;avg_vel_2(2)];
end 

% Xunit_2 = x(1,:);
% Yunit_2 = x(2,:);
% Hfinal_2 = atan2(u(2,:),u(1,:));

Xunit_1_ex = Xunit_1;
Yunit_1_ex = Yunit_1;
Hfinal_1_ex = hinit_1*ones(2*total_t-1,1);

Xunit_3_ex = Xunit_3;
Yunit_3_ex = Yunit_3;
Hfinal_3_ex = hinit_3*ones(2*total_t-1,1);

Xunit_4_ex = Xunit_4;
Yunit_4_ex = Yunit_4;
Hfinal_4_ex = hinit_4*ones(2*total_t-1,1);;

Xunit_2_ex = Xunit_2;
Yunit_2_ex = Yunit_2;
Hfinal_2_ex = hinit_2*ones(2*total_t-1,1);;
% th_ex = 0:pi/39:pi;

xunit_1_ex = xunit_1;
yunit_1_ex = yunit_1;
xunit_3_ex = xunit_3;
yunit_3_ex = yunit_3;
xunit_4_ex = xunit_4;
yunit_4_ex = yunit_4;

xunit_2_ex = xunit_2;
yunit_2_ex = yunit_2;

for i=2:total_t %i=1 corresponds to th_ex=0 which is same as th=2*pi
    xdel_1_ex = avg_vel_1(1)*del_t;
    ydel_1_ex = avg_vel_1(2)*del_t;
    
    xdel_3_ex = avg_vel_3(1)*del_t;
    ydel_3_ex = avg_vel_3(2)*del_t;
    
    xdel_4_ex = avg_vel_4(1)*del_t;
    ydel_4_ex = avg_vel_4(2)*del_t;
    
    xdel_2_ex = avg_vel_2(1)*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2_ex = avg_vel_2(2)*del_t;
    
    xnow_1_ex = xdel_1_ex + xunit_1_ex;
    ynow_1_ex = ydel_1_ex + yunit_1_ex;
    xunit_1_ex = xnow_1_ex;
    yunit_1_ex = ynow_1_ex;
    Xunit_1_ex = [Xunit_1_ex,xunit_1_ex];
    Yunit_1_ex = [Yunit_1_ex,yunit_1_ex];
    
    xnow_3_ex = xdel_3_ex + xunit_3_ex;
    ynow_3_ex = ydel_3_ex + yunit_3_ex;
    xunit_3_ex = xnow_3_ex;
    yunit_3_ex = ynow_3_ex;
    Xunit_3_ex = [Xunit_3_ex,xunit_3_ex];
    Yunit_3_ex = [Yunit_3_ex,yunit_3_ex];
    
    xnow_4_ex = xdel_4_ex + xunit_4_ex;
    ynow_4_ex = ydel_4_ex + yunit_4_ex;
    xunit_4_ex = xnow_4_ex;
    yunit_4_ex = ynow_4_ex;
    Xunit_4_ex = [Xunit_4_ex,xunit_4_ex];
    Yunit_4_ex = [Yunit_4_ex,yunit_4_ex];
    
    xnow_2_ex = xdel_2_ex + xunit_2_ex;
    ynow_2_ex = ydel_2_ex + yunit_2_ex;
    xunit_2_ex = xnow_2_ex;
    yunit_2_ex = ynow_2_ex;
    Xunit_2_ex = [Xunit_2_ex,xunit_2_ex];
    Yunit_2_ex = [Yunit_2_ex,yunit_2_ex];
end
