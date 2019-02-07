clear;
s_1 = [5,2];
g_1 = [-5,-2];

s_2 = [-5,0];
g_2 = [5,0];

total_t = 20; %treat it more like step numbers
del_t = 1;
Xunit_1 = s_1(1):(g_1(1)-s_1(1))/(total_t-1):g_1(1);
Yunit_1 = [s_1(2),1.7,1.3,1.0,0.8,0.6,0.3,0.1,zeros(1,total_t-16),-0.1,-0.3,-0.6,-0.8,-1.0,-1.3,-1.7,g_1(2)];

Xunit_2 = s_2(1):(g_2(1)-s_2(1))/(total_t-1):g_2(1);
%Yunit_2 = s_2(2):(g_2(2)-s_2(2))/(total_t-1):g_2(2);
Yunit_2 = zeros(1,total_t);
Xunit_2_ex = Xunit_2;
Yunit_2_ex = Yunit_2;
for i=2:total_t
    Vxunit_1(i-1) = (Xunit_1(i) - Xunit_1(i-1));
    Vyunit_1(i-1) = (Yunit_1(i) - Yunit_1(i-1));
    Hfinal_1(i-1) = atan2(Vyunit_1(i-1),Vxunit_1(i-1));
    
    Vxunit_2(i-1) = (Xunit_2(i) - Xunit_2(i-1));
    Vyunit_2(i-1) = (Yunit_2(i) - Yunit_2(i-1));
    Hfinal_2(i-1) = atan2(Vyunit_2(i-1),Vxunit_2(i-1));
end
Hfinal_1(i) = Hfinal_1(i-1);
Hfinal_2(i) = Hfinal_2(i-1);
Hfinal_2_ex = Hfinal_2;
for i=2:total_t
    xunit_2 = Xunit_2_ex(end) + Vxunit_2(end)*del_t;
    yunit_2 = Yunit_2_ex(end) + Vyunit_2(end)*del_t;
    Xunit_2_ex = [Xunit_2_ex,xunit_2];
    Yunit_2_ex = [Yunit_2_ex,yunit_2];
    Hfinal_2_ex = [Hfinal_2_ex, Hfinal_2(end)];
end
%%footprint of obstacle/unit 1
%hinit_1 = 0; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;


%%footprint of robot/unit 2
%hinit_2 = pi/4; % initial heading for unit 2
b_2 = 0.75;
l_2 = 1.5; 
L = 1; %rear to front distance
d_rear = L/2;

Xa = [];
Ya = [];
% figure;
% axis([Xunit_1(end) Xunit_1(1) Yunit_1(end) Yunit_1(1)]);
% for i = 1:total_t
%     [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
%     [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i)+d_rear*cos(Hfinal_2(i)),Yunit_2(i)+d_rear*sin(Hfinal_2(i)));
%     Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
%     Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
%     %[xa,ya] = polybool('union',Poly_0(:,1),Poly_0(:,2),Poly_1(:,1),Poly_1(:,2));
%     hold on;
%     plot(Poly_1(:,1),Poly_1(:,2));
%     
%     hold on;
%     plot(Poly_2(:,1),Poly_2(:,2));
%     
%     if(i>1)
%         [xa,ya] = polybool('union',Poly_0(:,1),Poly_0(:,2),Poly_1(:,1),Poly_1(:,2));
%         Xa = [Xa;xa];
%         Ya = [Ya;ya];
%     end
%     Poly_0 = Poly_1;
% end
Ar = zeros(length(Xunit_1),length(Xunit_2));
ar = [];
for i = 1:length(Xunit_2)
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i)+d_rear*cos(Hfinal_2(i)),Yunit_2(i)+d_rear*sin(Hfinal_2(i)));
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    for j = 1:length(Xunit_1)
        [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(j),Xunit_1(j),Yunit_1(j));
        Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
        [x_int,y_int] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
        arr = polyarea(x_int,y_int);
        ar = [ar,arr];
        Ar(j,i) = arr;
    end
end

ind = find(sum(Ar,2));
% figure
% axis([Xunit_1(end) Xunit_1(1) Yunit_1(end) Yunit_1(1)]);
% for i = 1:length(ind)
% % for i = ind(12):ind(end)
%     [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(ind(i)),Xunit_1(ind(i)),Yunit_1(ind(i)));
%     [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(ind(i)),Xunit_2(ind(i))+d_rear*cos(Hfinal_2(ind(i))),Yunit_2(ind(i))+d_rear*sin(Hfinal_2(ind(i))));
%     Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
%     Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
%     %[xa,ya] = polybool('union',Poly_0(:,1),Poly_0(:,2),Poly_1(:,1),Poly_1(:,2));
%     hold on;
%     plot(Poly_1(:,1),Poly_1(:,2));
% end
Hfinal_2 = Hfinal_2';
Hfinal_2_ex = Hfinal_2_ex';