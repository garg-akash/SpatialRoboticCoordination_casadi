function [lelo, w_s, lbw_s, ubw_s, w0_s, discrete_s, g_s, lbg_s, ubg_s] = dbg_CheckSide(rect1,rect2,x_rot,y_rot,x_ref,y_ref,l)

addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
% compute projection of vertex onto perpendicular edge using dot
% product => sign gives direction
w_s={};
w0_s = [];
lbw_s = [];
ubw_s = [];
discrete_s = [];
g_s={};
lbg_s = [];
ubg_s = [];
ns = 4;
BIG = 100000;
side1 = zeros(1,4);
side2 = zeros(1,4);
ind = 0;
for k=1:4
    side1(k) = (x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref))
end
for k=1:4
    side2(k) = (x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref))
end

% check if vertices are on different sides of the edge
%     if ( (min(side1) >= 0 && max(side2) <= 0) || (max(side1) <= 0 && min(side2) >= 0) || ...
%             (max(side1) < min(side2)) || (max(side2) < min(side1)) )

% mx_s1 = SX.sym(['mx_s1_' num2str(l)]);   %calculates max of side1
% mn_s1 = SX.sym(['mn_s1_' num2str(l)]);   %calculates min of side1
% dmx1 = SX.sym(['dmx1_' num2str(l)], ns);   %decision var for max of side1
% dmn1 = SX.sym(['dmn1_' num2str(l)], ns);   %decision var for min of side1
% 
% w_s = {mx_s1 dmx1};
% lbw_s = [-inf; zeros(ns,1)];
% ubw_s = [inf; ones(ns,1)];
% w0_s = [0; ones(ns,1)];
% discrete_s = [0; ones(4,1)];
% disp('At First')
% length(w_s)
% length(discrete_s)
% g_s = {mx_s1-side1(1) mx_s1-side1(2) mx_s1-side1(3) mx_s1-side1(4)};% ...
% g_s = {mx_s1-side1(1)-BIG*(1-dmx1(1)) mx_s1-side1(2)-BIG*(1-dmx1(2)) mx_s1-side1(3)-BIG*(1-dmx1(3)) ...
%     mx_s1-side1(4)-BIG*(1-dmx1(4)) dmx1(1)+dmx1(2)+dmx1(3)+dmx1(4)};
% lbg_s = [0; 0; 0; 0; -inf; -inf; -inf; -inf; 1];
% ubg_s = [inf; inf; inf; inf; 0; 0; 0; 0; 1];
% 
% w_s = {w_s{:} mn_s1 dmn1};
% lbw_s = [lbw_s; -inf; zeros(ns,1)];
% ubw_s = [ubw_s; inf; ones(ns,1)];
% w0_s = [w0_s; 0; ones(ns,1)];
% discrete_s = [discrete_s; 0; ones(4,1)];
% g_s = {g_s{:} mn_s1-side1(1) mn_s1-side1(2) mn_s1-side1(3) mn_s1-side1(4) ...
%     mn_s1-side1(1)+BIG*(1-dmn1(1)) mn_s1-side1(2)+BIG*(1-dmn1(2)) mn_s1-side1(3)+BIG*(1-dmn1(3)) ...
%     mn_s1-side1(4)+BIG*(1-dmn1(4)) dmn1(1)+dmn1(2)+dmn1(3)+dmn1(4)};
% lbg_s = [lbg_s; -inf; -inf; -inf; -inf; 0; 0; 0; 0; 1];
% ubg_s = [ubg_s; 0; 0; 0; 0; inf; inf; inf; inf; 1];
% % mx_s1 = max(side1);
% % mn_s1 = min(side1);
% mx_s2 = SX.sym(['mx_s2_' num2str(l)]);   %calculates max of side2
% mn_s2 = SX.sym(['mn_s2_' num2str(l)]);   %calculates min of side2
% dmx = SX.sym(['dmx_' num2str(l)], ns);   %decision var for max of side2
% dmn = SX.sym(['dmn_' num2str(l)], ns);   %decision var for min of side2
% 
% w_s = {w_s{:} mx_s2 dmx};
% lbw_s = [lbw_s; -inf; zeros(ns,1)];
% ubw_s = [ubw_s; inf; ones(ns,1)];
% w0_s = [w0_s; 0; ones(ns,1)];
% discrete_s = [discrete_s; 0; ones(4,1)];
% g_s = {g_s{:} mx_s2-side2(1) mx_s2-side2(2) mx_s2-side2(3) mx_s2-side2(4)};% ...
% g_s = {g_s{:} mx_s2-side2(1)-BIG*(1-dmx(1)) mx_s2-side2(2)-BIG*(1-dmx(2)) mx_s2-side2(3)-BIG*(1-dmx(3)) ...
%     mx_s2-side2(4)-BIG*(1-dmx(4)) dmx(1)+dmx(2)+dmx(3)+dmx(4)};
% lbg_s = [lbg_s; 0; 0; 0; 0; -inf; -inf; -inf; -inf; 1];
% ubg_s = [ubg_s; inf; inf; inf; inf; 0; 0; 0; 0; 1];
% 
% w_s = {w_s{:} mn_s2 dmn};
% lbw_s = [lbw_s; -inf; zeros(ns,1)];
% ubw_s = [ubw_s; inf; ones(ns,1)];
% w0_s = [w0_s; 0; ones(ns,1)];
% discrete_s = [discrete_s; 0; ones(4,1)];
% g_s = {g_s{:} mn_s2-side2(1) mn_s2-side2(2) mn_s2-side2(3) mn_s2-side2(4) ...
%     mn_s2-side2(1)+BIG*(1-dmn(1)) mn_s2-side2(2)+BIG*(1-dmn(2)) mn_s2-side2(3)+BIG*(1-dmn(3)) ...
%     mn_s2-side2(4)+BIG*(1-dmn(4)) dmn(1)+dmn(2)+dmn(3)+dmn(4)};
% lbg_s = [lbg_s; -inf; -inf; -inf; -inf; 0; 0; 0; 0; 1];
% ubg_s = [ubg_s; 0; 0; 0; 0; inf; inf; inf; inf; 1];

mx_s1 = max(side1);
mn_s1 = min(side1);
mx_s2 = max(side2);
mn_s2 = min(side2);
dcn_21 = SX.sym(['dcn21_' num2str(l)]);   %decision var to compare mn_s2 with mx_s1

w_s = {w_s{:} dcn_21};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 1];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mn_s2-mx_s1-BIG*dcn_21 mn_s2-mx_s1+BIG*(1-dcn_21)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

dcn_12 = SX.sym(['dcn12_' num2str(l)]);   %decision var to compare mn_s1 with mx_s2

w_s = {w_s{:} dcn_12};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 1];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mn_s1-mx_s2-BIG*dcn_12 mn_s1-mx_s2+BIG*(1-dcn_12)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

dcx_21 = SX.sym(['dcx21_' num2str(l)]);   %decision var to compare mx_s2 with mx_s1

w_s = {w_s{:} dcx_21};
lbw_s = [lbw_s; 0];
ubw_s = [ubw_s; 1];
w0_s = [w0_s; 1];
discrete_s = [discrete_s; 1];
g_s = {g_s{:} mx_s2-mx_s1-BIG*dcx_21 mx_s2-mx_s1+BIG*(1-dcx_21)};
lbg_s = [lbg_s; -inf; 0];
ubg_s = [ubg_s; 0; inf];

lelo = ((mx_s1 - mn_s2)*dcx_21 + (mx_s2 - mn_s1)*(1 - dcx_21))*(1-dcn_21)*(1-dcn_12);

% if( (max(side1) < min(side2)) || (max(side2) < min(side1)))
%     ind = 1;
%     lelo = 0;
% else
%     if (max(side2) > max(side1))
%         ind = 2;
%         lelo = max(side1) - min(side2);
%     else
%         ind = 3;
%         lelo = max(side2) - min(side1);
%     end
% end
% fprintf('Indicator is %d',ind);

end