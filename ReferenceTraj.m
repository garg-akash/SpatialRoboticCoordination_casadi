%File to generate reference trajectory 
c_1 = [0,3.5];
c_2 = [0,-3.5];
r = 4;
total_t = 40;
avg_vel = 2*pi*r/total_t;
th = pi:pi/39:2*pi;
del_t = total_t/(2*length(th));

%%footprint of obstacle/unit 1
hinit_1 = 3*pi/2; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;

%%footprint of robot/unit 2
hinit_2 = pi/2; % initial heading for unit 2
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
xnow_1 = c_1(1)-r;
ynow_1 = c_1(2);
xnow_2 = c_2(1)-r;
ynow_2 = c_2(2);

for i=1:length(th)
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    hfinal_1 = th(i) + hinit_1;
    Hfinal_1 = [Hfinal_1,hfinal_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    hfinal_2 = th(i) + hinit_2;
    Hfinal_2 = [Hfinal_2,hfinal_2];
    
    xdel_1 = avg_vel*(-sin(th(i)))*del_t;
    ydel_1 = avg_vel*(cos(th(i)))*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_2 = -avg_vel*(-sin(2*pi-th(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = -avg_vel*(cos(2*pi-th(i)))*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;-avg_vel*(-sin(2*pi-th(i)))];
    Vyunit_2 = [Vyunit_2;-avg_vel*(cos(2*pi-th(i)))];
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
th_ex = 0:pi/39:pi;

xunit_1_ex = xunit_1;
yunit_1_ex = yunit_1;
xunit_2_ex = xunit_2;
yunit_2_ex = yunit_2;
for i=2:length(th_ex) %i=1 corresponds to th_ex=0 which is same as th=2*pi
    xdel_1_ex = avg_vel*(-sin(th_ex(i)))*del_t;
    ydel_1_ex = avg_vel*(cos(th_ex(i)))*del_t;
    
    xdel_2_ex = -avg_vel*(-sin(2*pi-th_ex(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2_ex = -avg_vel*(cos(2*pi-th_ex(i)))*del_t;
    
    xnow_1_ex = xdel_1_ex + xunit_1_ex;
    ynow_1_ex = ydel_1_ex + yunit_1_ex;
    xunit_1_ex = xnow_1_ex;
    yunit_1_ex = ynow_1_ex;
    Xunit_1_ex = [Xunit_1_ex,xunit_1_ex];
    Yunit_1_ex = [Yunit_1_ex,yunit_1_ex];
    Hfinal_1_ex = [Hfinal_1_ex,(th_ex(i) + hinit_1)];
    xnow_2_ex = xdel_2_ex + xunit_2_ex;
    ynow_2_ex = ydel_2_ex + yunit_2_ex;
    xunit_2_ex = xnow_2_ex;
    yunit_2_ex = ynow_2_ex;
    Xunit_2_ex = [Xunit_2_ex,xunit_2_ex];
    Yunit_2_ex = [Yunit_2_ex,yunit_2_ex];
    Hfinal_2_ex = [Hfinal_2_ex,(th_ex(i) + hinit_2)];
end

Ar = [];
Ar_ex = [];
for i = 1:length(th)
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i),Yunit_2(i));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    [xa,ya] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
    arr = polyarea(xa,ya);
    Ar = [Ar,arr];
end
Ar_ex = Ar;
for i=2:length(th_ex)
    Ar_ex = [Ar_ex,0];
end
