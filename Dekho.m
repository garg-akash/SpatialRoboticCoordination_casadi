%This file load path text file to a MATLAB matrix; Obstacle in Matrix A and
%Robot in Matrix B
% clear;
clc;
A = importdata('TextFiles/enter/path1.txt');
B = importdata('TextFiles/enter/constraints_traj.txt');
N = 9; %planning horizon
nv = 3;
Na = 1; %Start of path 1
Nb = 39; %End of path 2
Nsim = 39;
skip = 0;
reverse = 0;
l_1 = 2.3;
b_1 = 0.7;
l_2 = 2.3;
b_2 = 0.7;
L = 1.19; %rear to front distance
d_rear = L/2;
A = A(Na:end,:);
B = B(1:Nb,:);
A = A(1:(1+skip):end,:);
del_t = 0.5*(1+skip);
if (skip>0 && reverse==0)
    disp('First loop');
    B = B(1:(1+skip):end,:);
    for i=1:length(B)-1
        del_x = B(i+1,1)-B(i,1);
        del_y = B(i+1,2)-B(i,2);
        B(i,3) = atan2(del_y,del_x);
        B(i,5) = sqrt(del_x^2 + del_y^2)/del_t;
    end
    B(i+1,3) = B(i,3);
    B(i+1,5) = 0;
    for i=1:length(B)-1
        del_th = B(i+1,3)-B(i,3);
        B(i,4) = atan2(del_th*L/(B(i,5)*del_t),1);
    end
    B(i+1,4) = 0;
    for i=1:length(B)-1
        B(i,6) = (B(i+1,4)-B(i,4))/del_t;
    end
    B(i+1,6) = 0;
end

if (skip==0 && reverse==1)
    disp('Second loop');
%     B = flip(B);
%     B(1:end-1,5) = B(2:end,5);
%     B(end,5) = 0;   
%     for i=1:length(B)-1
%         del_x = -B(i+1,1)+B(i,1);
%         del_y = -B(i+1,2)+B(i,2);
%         B(i,3) = atan2(del_y,del_x);
%     end
%     B(i+1,3) = B(i,3);
%     for i=1:length(B)-1
%         del_th = -B(i+1,3)+B(i,3);
%         B(i,4) = atan2(del_th*L/(B(i,5)*del_t),1);
%     end
%     B(i+1,4) = 0;
%     for i=1:length(B)-1
%         B(i,6) = (-B(i+1,4)+B(i,4))/del_t;
%     end
%     B(i+1,6) = 0;
    
%     B = flip(B);
%     for i=1:length(B)-1
%         del_x = B(i+1,1)-B(i,1);
%         del_y = B(i+1,2)-B(i,2);
%         B(i,3) = atan2(del_y,del_x)+ 2*pi*(del_y<0);
%         B(i,5) = sqrt(del_x^2 + del_y^2)/del_t;
%     end
%     B(i+1,3) = B(i,3);
%     B(i+1,5) = 0;
%     for i=1:length(B)-1
%         del_th = B(i+1,3)-B(i,3);
%         B(i,4) = atan2(del_th*L/(B(i,5)*del_t),1);
%     end
%     B(i+1,4) = 0;
%     for i=1:length(B)-1
%         B(i,6) = (B(i+1,4)-B(i,4))/del_t;
%     end
%     B(i+1,6) = 0;


    B = flip(B);
    B(1:end-1,5) = B(2:end,5);
    B(end,5) = 0; 
    B(1:end-1,6) = B(2:end,6);
    B(end,6) = 0; 
end

if (skip>0 && reverse==1)
    disp('Third loop');
    B = flip(B);
    B = B(1:(1+skip):end,:);
    for i=1:length(B)-1
        del_x = -B(i+1,1)+B(i,1);
        del_y = -B(i+1,2)+B(i,2);
        B(i,3) = atan2(del_y,del_x);
        B(i,5) = sqrt(del_x^2 + del_y^2)/del_t;
    end
    B(i+1,3) = B(i,3);
    B(i+1,5) = 0;
    for i=1:length(B)-1
        del_th = -B(i+1,3)+B(i,3);
        B(i,4) = atan2(del_th*L/(B(i,5)*del_t),1);
    end
    B(i+1,4) = 0;
    for i=1:length(B)-1
        B(i,6) = (-B(i+1,4)+B(i,4))/del_t;
    end
    B(i+1,6) = 0;
end
% if (reverse)
%     B = flip(B);
% %     B(1:end-1,5) = B(2:end,5);
% %     B(end,5) = 0;
% %     B(1:end-1,6) = B(2:end,6);
% %     B(end,6) = 0;
% end


% B_orig = B;
% if (skip > 0)
%     for i=1:length(B)-1
% %         B(i,7) = (B(i,1)-B(i+1,1))/(cos(B(i,3))*del_t);
% %         B(i,8) = (B(i,2)-B(i+1,2))/(sin(B(i,3))*del_t);
% %         B(i,9) = (B(i,4)-B(i+1,4))/del_t;
%         B(i,5) = sqrt( (B(i,1)-B(i+1,1))^2 + (B(i,2)-B(i+1,2))^2 )/del_t;
%         B(i,6) = (B(i,4)-B(i+1,4))/del_t;
%     end
%     B(length(B),5) = 0;
%     B(length(B),6) = 0;
% end
% del_t = 0.75;
% %%
% B_d = B(:,1:2);
% if (reverse)
%     B_d = flip(B_d);
% end
% for i=1:length(B)-1
%     del_x = B_d(i+1,1)-B_d(i,1);
%     del_y = B_d(i+1,2)-B_d(i,2);
%     B_d(i,3) = atan2(del_y,del_x);
%     B_d(i,5) = sqrt(del_x^2 + del_y^2)/del_t;
% end
% B_d(i+1,3) = B_d(i,3);
% B_d(i+1,5) = 0;
% for i=1:length(B)-1
%     del_th = B_d(i+1,3)-B_d(i,3);
%     B_d(i,4) = atan2(del_th*L/B_d(i,5),1);
% end
% B_d(i+1,4) = 0;
% for i=1:length(B)-1
%     B_d(i,6) = (B_d(i+1,4)-B_d(i,4))/del_t;
% end
% B_d(i+1,6) = 0;
% B = B_d;
% %%
total_t = length(B);
Xunit_1 = A(:,1);
Yunit_1 = A(:,2);
Hfinal_1 = A(:,3);

Xunit_2 = B(:,1);
Yunit_2 = B(:,2);
Hfinal_2 = B(:,3);

figure;
% hold on
%axis([Xunit_1(end) Xunit_1(1) Yunit_1(end) Yunit_1(1)]);
for i = 1:min(total_t,length(A))
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i)+d_rear*cos(Hfinal_2(i)),Yunit_2(i)+d_rear*sin(Hfinal_2(i)));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    %[xa,ya] = polybool('union',Poly_0(:,1),Poly_0(:,2),Poly_1(:,1),Poly_1(:,2));
    hold on;
    plot(Poly_1(:,1),Poly_1(:,2));
    
    hold on;
    plot(Poly_2(:,1),Poly_2(:,2));
end
Ar = zeros(length(Xunit_1),length(Xunit_2));
ar = [];
for i = 1:length(Xunit_1)
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i)+d_rear*cos(Hfinal_1(i)),Yunit_1(i)+d_rear*sin(Hfinal_1(i)));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    for j = 1:length(Xunit_2)
        [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(j),Xunit_2(j)+d_rear*cos(Hfinal_2(j)),Yunit_2(j)+d_rear*sin(Hfinal_2(j)));
        Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
        [x_int,y_int] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
        arr = polyarea(x_int,y_int);
        ar = [ar,arr];
        Ar(i,j) = arr;
    end
end

ind = find(sum(Ar,2)); %represents indices of robot_1/obstacle that have overlap with robot_2
figure
%axis([Xunit_1(end) Xunit_1(1) Yunit_1(end) Yunit_1(1)]);
for i = 1:length(ind)
% for i = ind(12):ind(end)
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(ind(i)),Xunit_1(ind(i)),Yunit_1(ind(i)));
    %[AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(ind(i)),Xunit_2(ind(i))+d_rear*cos(Hfinal_2(ind(i))),Yunit_2(ind(i))+d_rear*sin(Hfinal_2(ind(i))));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    %Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    %[xa,ya] = polybool('union',Poly_0(:,1),Poly_0(:,2),Poly_1(:,1),Poly_1(:,2));
    hold on;
    plot(Poly_1(:,1),Poly_1(:,2));
end

% Xunit_2_ex = B(:,1);
% Yunit_2_ex = B(:,2);
% Hfinal_2_ex = B(:,3);
Xunit_2_ex = [Xunit_2;Xunit_2(end)*ones(N,1)];
Yunit_2_ex = [Yunit_2;Yunit_2(end)*ones(N,1)];
Hfinal_2_ex = [Hfinal_2;Hfinal_2(end)*ones(N,1)];

%Make Hfinal_2 a column vector 
Hfinal_2 = Hfinal_2';
Hfinal_2_ex = Hfinal_2_ex';

vel_0 = [B(:,5);repmat(B(end,5),N,1)];
omg_0 = [B(:,6);repmat(B(end,6),N,1)];

c_A0 = importdata('TextFiles/robotLab/constraints_A0.txt');
c_A1 = importdata('TextFiles/robotLab/constraints_A1.txt');
c_b = importdata('TextFiles/robotLab/constraints_b.txt');
c_th = importdata('TextFiles/robotLab/constraints_th.txt');
if (reverse)
    c_A0 = flip(c_A0);
    c_A1 = flip(c_A1);
    c_b = flip(c_b);
    c_th = flip(c_th);
end
c_A0 = c_A0(1:Nb,:);
c_A1 = c_A1(1:Nb,:);
c_b = c_b(1:Nb,:);
c_th = c_th(1:Nb,:);
c_A0 = c_A0(1:(1+skip):end,:);
c_A1 = c_A1(1:(1+skip):end,:);
c_b = c_b(1:(1+skip):end,:);
c_th = c_th(1:(1+skip):end,:);

cons_A0 = [c_A0;repmat(c_A0(end,1:4),N,1)];
cons_A1 = [c_A1;repmat(c_A1(end,1:4),N,1)];
cons_b = [c_b;repmat(c_b(end,1:4),N,1)];
cons_th = [c_th;repmat(c_th(end,1:2),N,1)];