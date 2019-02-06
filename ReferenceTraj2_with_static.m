%File to generate reference trajectory 1-Obs1; 2-Robot, 3-Obs2
s_1 = [10,-0.5];
g_1 = [0,-0.5];
s_2 = [0,0];
g_2 = [10,0];

s_3 = [5,0.75];
g_3 = s_3;

total_t = 40;
%th = pi:pi/39:2*pi;
del_t = 1;
avg_vel = (g_1 - s_1)/(total_t*del_t);

%%footprint of obstacle/unit 1
hinit_1 = pi; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;

%%footprint of obstacle/unit 3
b_3 = 1;
l_3 = 1.5;

%%footprint of robot/unit 2
hinit_2 = 0; % initial heading for unit 2
b_2 = 0.75;
l_2 = 1.5; 

Xunit_1=[];
Yunit_1=[];
Hfinal_1=[];
Xunit_2=[];
Yunit_2=[];
Hfinal_2=[];
Vxunit_2=[];
Vyunit_2=[];
xnow_1 = s_1(1);
ynow_1 = s_1(2);
xnow_2 = s_2(1);
ynow_2 = s_2(2);

Xunit_3 = s_3(1)*ones(1,40);
Yunit_3 = s_3(2)*ones(1,40);
Hfinal_3 = zeros(1,40);
for i=1:total_t
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    hfinal_1 = hinit_1;
    Hfinal_1 = [Hfinal_1,hfinal_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    hfinal_2 = hinit_2;
    Hfinal_2 = [Hfinal_2,hfinal_2];
    
    xdel_1 = avg_vel(1)*del_t;
    ydel_1 = avg_vel(2)*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_2 = -avg_vel(1)*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = -avg_vel(2)*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;-avg_vel(1)];
    Vyunit_2 = [Vyunit_2;-avg_vel(2)];
end 

% Xunit_2 = x(1,:);
% Yunit_2 = x(2,:);
% Hfinal_2 = atan2(u(2,:),u(1,:));

Xunit_1_ex = Xunit_1;
Yunit_1_ex = Yunit_1;
Hfinal_1_ex = Hfinal_1;

Xunit_2_ex = Xunit_2;
Yunit_2_ex = Yunit_2;
Hfinal_2_ex = Hfinal_2;
% th_ex = 0:pi/39:pi;

xunit_1_ex = xunit_1;
yunit_1_ex = yunit_1;
xunit_2_ex = xunit_2;
yunit_2_ex = yunit_2;

Xunit_3_ex = s_3(1)*ones(1,79);
Yunit_3_ex = s_3(2)*ones(1,79);
Hfinal_3_ex = zeros(1,79);
for i=2:total_t %i=1 corresponds to th_ex=0 which is same as th=2*pi
    xdel_1_ex = avg_vel(1)*del_t;
    ydel_1_ex = avg_vel(2)*del_t;
    
    xdel_2_ex = -avg_vel(1)*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2_ex = -avg_vel(2)*del_t;
    
    xnow_1_ex = xdel_1_ex + xunit_1_ex;
    ynow_1_ex = ydel_1_ex + yunit_1_ex;
    xunit_1_ex = xnow_1_ex;
    yunit_1_ex = ynow_1_ex;
    Xunit_1_ex = [Xunit_1_ex,xunit_1_ex];
    Yunit_1_ex = [Yunit_1_ex,yunit_1_ex];
    Hfinal_1_ex = [Hfinal_1_ex, hinit_1];
    xnow_2_ex = xdel_2_ex + xunit_2_ex;
    ynow_2_ex = ydel_2_ex + yunit_2_ex;
    xunit_2_ex = xnow_2_ex;
    yunit_2_ex = ynow_2_ex;
    Xunit_2_ex = [Xunit_2_ex,xunit_2_ex];
    Yunit_2_ex = [Yunit_2_ex,yunit_2_ex];
    Hfinal_2_ex = [Hfinal_2_ex, hinit_2];
end
