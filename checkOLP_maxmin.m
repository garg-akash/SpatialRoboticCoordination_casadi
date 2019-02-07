function [olp, w_s, lbw_s, ubw_s, w0_s, discrete_s, g_s, lbg_s, ubg_s] = ...
                checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l)

addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
BIG = 10000;
side1 = SX.zeros(1,ns);
side2 = SX.zeros(1,ns);

for k=1:4
    side1(k) = x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref);
end
for k=1:4
    side2(k) = x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref);
end

mx_s1 = max(side1);
mn_s1 = min(side1);

mx_s2 = max(side2);
mn_s2 = min(side2);

d1 = SX.sym(['d1_' num2str(l)]);   %decision var to compare mn_s2 with mx_s1

w_s = {d1};
lbw_s = [0];
ubw_s = [1];
w0_s = [0];
discrete_s = [1];
g_s = {mn_s2-mx_s1-BIG*d1 mn_s2-mx_s1+BIG*(1-d1)};
lbg_s = [-inf; 0];
ubg_s = [0; inf];

d2 = SX.sym(['d2_' num2str(l)]);   %decision var to compare mn_s1 with mx_s2

w_s = {w_s{:} d2};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 0];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mn_s1-mx_s2-BIG*d2 mn_s1-mx_s2+BIG*(1-d2)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

d3 = SX.sym(['d3_' num2str(l)]);   %decision var to compare mx_s1 with mx_s2

w_s = {w_s{:} d3};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 0];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mx_s1-mx_s2-BIG*d3 mx_s1-mx_s2+BIG*(1-d3)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

olp = ((mx_s2 - mn_s1)*d3 + (mx_s1 - mn_s2)*(1-d3))*(1-d1)*(1-d2);
