function [olp, w_s, lbw_s, ubw_s, w0_s, discrete_s, g_s, lbg_s, ubg_s] = ...
                checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l, ag, j)

addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
BIG = 10000;
side1 = SX.zeros(1,ns);
side2 = SX.zeros(1,ns);
for k=1:4
    side1(k) = (x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref))/sqrt(x_rot^2 + y_rot^2);
    %side1(k) = (x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref));
end
for k=1:4
    side2(k) = (x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref))/sqrt(x_rot^2 + y_rot^2);
    %side2(k) = (x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref));
end
mx_s1 = max(side1);
mn_s1 = min(side1);

mx_s2 = max(side2);
mn_s2 = min(side2);

% if(ag == 1)
%     if(j == 2)
%         mx_s1 = 2.1;
%         mn_s1 = 0;
%     elseif(j == 3)
%         mx_s1 = 0.6;
%         mn_s1 = 0;
%     end
%     for k=1:4
%         side2(k) = (x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref))/sqrt(x_rot^2 + y_rot^2);
%     end
%     mx_s2 = max(side2);
%     mn_s2 = min(side2);
% elseif (ag == 2)
%     if(j == 2)
%         mx_s2 = 2.1;
%         mn_s2 = 0;
%     elseif(j == 3)
%         mx_s2 = 0.6;
%         mn_s2 = 0;
%     end
%     for k=1:4
%         side1(k) = (x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref))/sqrt(x_rot^2 + y_rot^2);
%     end
%     mx_s1 = max(side1);
%     mn_s1 = min(side1);
% end
d1 = SX.sym(['d1_' num2str(l)]);   %decision var to compare mn_s2 with mx_s1
%d1 is going to be 0 when mn2<mx1, 1 otherwise
w_s = {d1};
lbw_s = [0];
ubw_s = [1];
w0_s = [0];
discrete_s = [1];
g_s = {mn_s2-mx_s1-BIG*d1 mn_s2-mx_s1+BIG*(1-d1)};
lbg_s = [-inf; 0];
ubg_s = [0; inf];

d2 = SX.sym(['d2_' num2str(l)]);   %decision var to compare mn_s1 with mx_s2
%d2 is going to be 0 when mn1<mx2, 1 otherwise
w_s = {w_s{:} d2};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 0];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mn_s1-mx_s2-BIG*d2 mn_s1-mx_s2+BIG*(1-d2)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

d3 = SX.sym(['d3_' num2str(l)]);   %decision var to compare mx_s1 with mx_s2
%d3 is 0 when mx_s1<mx_s2, 1 otherwise
w_s = {w_s{:} d3};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 0];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mx_s1-mx_s2-BIG*d3 mx_s1-mx_s2+BIG*(1-d3)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

d4 = SX.sym(['d4_' num2str(l)]);   %decision var to compare mx_s1 with mx_s2
%d4 is 0 when mn_s1<mn_s2, 1 otherwise
w_s = {w_s{:} d4};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 0];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mn_s1-mn_s2-BIG*d4 mn_s1-mn_s2+BIG*(1-d4)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

olp = (((mx_s2 - mn_s2)*(1-d4) + (mx_s2-mn_s1)*d4)*d3 ...
    + ((mx_s1 - mn_s2)*(1-d4) + (mx_s1 - mn_s1)*d4)*(1-d3))*(1-d1)*(1-d2) + 0*d1*d2;
% olp = ((mx_s2 - mn_s1)*d3 + (mx_s1 - mn_s2)*(1-d3))*(1+q1)*(1+q2);