function[AG,BG,CG,DG]=rectangle_plot(l,b,theta,x_centre,y_centre)

%% start from right hand side: 
%% 
%%  B---------A
%%  |         |
%%  |         |
%%  C---------D

%% co-ordinates of A:
X_A=b/2;Y_A=l/2;
angle_A=atan2(Y_A,X_A);

%% co-ordinates of B:
X_B=-b/2;Y_B=l/2;
angle_B=atan2(Y_B,X_B);
%% co-ordinates of C:
X_C=-b/2;Y_C=-l/2;
angle_C=atan2(Y_C,X_C);
%% co-ordinates of D:
X_D=b/2;Y_D=-l/2;
angle_D=atan2(Y_D,X_D);
%% tansform the co-ordinates to the global frame:
r=sqrt((l/2)^2+(b/2)^2);
X_AG=x_centre+r*cos(angle_A+theta-pi/2);
Y_AG=y_centre+r*sin(angle_A+theta-pi/2);

X_BG=x_centre+r*cos(angle_B+theta-pi/2);
Y_BG=y_centre+r*sin(angle_B+theta-pi/2);

X_CG=x_centre+r*cos(angle_C+theta-pi/2);
Y_CG=y_centre+r*sin(angle_C+theta-pi/2);

X_DG=x_centre+r*cos(angle_D+theta-pi/2);
Y_DG=y_centre+r*sin(angle_D+theta-pi/2);

AG = [X_AG,Y_AG];
BG = [X_BG,Y_BG];
CG = [X_CG,Y_CG];
DG = [X_DG,Y_DG];
% %% edge A-B:
% p1 = line([X_AG;X_BG],[Y_AG;Y_BG]);
% hold on;
% %% edge B-C:
% p2 = line([X_BG;X_CG],[Y_BG;Y_CG]);
% hold on;
% %% edge C-D:
% p3 = line([X_CG;X_DG],[Y_CG;Y_DG]);
% hold on;
% %% edge D-A:
% p4 = line([X_DG;X_AG],[Y_DG;Y_AG]);
% hold on;
% theta
% mid1_x=(X_AG+X_BG)/2;mid1_y=(Y_AG+Y_BG)/2;
% mid2_x=(X_CG+X_DG)/2;mid2_y=(Y_CG+Y_DG)/2;
% 
% %% line joining 2 midpoints:
% line([mid1_x;mid2_x],[mid1_y;mid2_y],'Color','red','LineStyle','--');



