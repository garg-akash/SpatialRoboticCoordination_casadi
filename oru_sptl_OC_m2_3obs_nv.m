function [v_out, th_out, x_out, y_out, d_values] = oru_sptl_OC_m2_3obs_nv(iter, N, del_t, r_st ...
                                , o_px, o_py, o_h, r_px, r_py, r_h, r_prev, o3_px, o3_py, o3_h ...
                                , o4_px, o4_py, o4_h, o5_px, o5_py, o5_h, o7_px, o7_py, o7_h ...
                                , l_1, b_1, l_2, b_2, l_3, b_3, l_4, b_4, l_5, b_5, l_7, b_7,g_2,nv,dGrad)
% To implement loop optimal control
% Xunit_2 = x_opt;
% Yunit_2 = y_opt;
% Hfinal_2 = atan2(u_opt(41:end),u_opt(1:40));
import casadi.*
nx  = 1;
nu  = 1;
h = del_t; %discretization step
ns = 4; %number of sides in polygon

%[AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,0.78,r_prev(1),r_prev(2)); %it is for circum circle, theta doesn't matter
%r_cent = (AG_2 + CG_2)/2;
r_cent = r_prev;
r_rad = sqrt(power(l_2,2)+power(b_2,2))/2;

Q = 0.1;
R = 0.9;
V = SX.sym('V',nu);
tht = SX.sym('tht',nu);
Px = SX.sym('Px',nx);
Py = SX.sym('Py',nx);

xdot = V(1)*cos(tht(1));
ydot = V(1)*sin(tht(1));

% Intergrator for x and y
fy = Function('fy', {Py,V,tht},{ydot});
fx = Function('fx', {Px,V,tht},{xdot});

% RK4
V    = MX.sym('V');
tht  = MX.sym('tht');
PX0  = MX.sym('PX0',nx);
PX   = PX0;

k = fx(PX,V,tht);
k1 = k;
k = fx(PX + h/2 * k1, V, tht);
k2 = k;
k = fx(PX + h/2 * k2, V, tht);
k3 = k;
k = fx(PX + h   * k3, V, tht);
k4 = k;
PX = PX + h/6*(k1   + 2*k2   + 2*k3   + k4);
RK4x = Function('RK4x',{PX0,V,tht},{PX});

% fy = Function('fy', {Py,V,th},{ydot});

% % RK4
PY0  = MX.sym('PY0',nx);
PY   = PY0;

k = fy(PY,V,tht);
k1 = k;
k = fy(PY + h/2 * k1, V, tht);
k2 = k;
k = fy(PY + h/2 * k2, V, tht);
k3 = k;
k = fy(PY + h   * k3, V, tht);
k4 = k;
PY = PY + h/6*(k1   + 2*k2   + 2*k3   + k4);
RK4y = Function('RK4y',{PY0,V,tht},{PY});


% Formulate NLP (use matrix graph)
%nv = 3; %Number of points of interest from all N points
V = SX.sym('V',nv);
tht = SX.sym('tht',nv);

% Objective function
w={};
w0 = [];
lbw = [];
ubw = [];
discrete = [];
J = 0;
g={};
lbg = [];
ubg = [];

% Get an expression for the cost and state at end
x_0 = r_st(1);
PX = x_0;
y_0 = r_st(2); 
PY = y_0;

w = {w{:} V tht};
lbw = [lbw; 0*ones(nv,1); -0.8+r_h(1:N/nv:N)];
ubw = [ubw; 0.6*ones(nv,1); 0.8+r_h(1:N/nv:N)];
w0 = [w0; 0.6*ones(nv,1); 0.7*ones(nv,1)];
discrete = [discrete; zeros(nv,1); zeros(nv,1)];
% g = {g{:} tht(2:end)-tht(1:end-1)};
% lbg = [lbg; -0.7854*ones(nv-1,1)];
% ubg = [ubg; 0.7854*ones(nv-1,1)];
l = 1; %It's a global variable to number the decision variables
olp_all = SX.zeros(2*ns*N,1);  %keep overlap value
id = 1;
for i = 1:N/nv:N
%     if (i==(N/nv-1) || i==(2*N/nv-1) || i==(3*N/nv-1) || i==(4*N/nv-1) || i==(5*N/nv-1) ...
%             || i==(6*N/nv-1) || i==(7*N/nv-1) || i==(8*N/nv-1) || i==(9*N/nv-1)|| i==(10*N/nv-1))
    flag = zeros(8,1);
    out_x = RK4x(PX,V(id),tht(id));
    PX = out_x;
    
    out_y = RK4y(PY,V(id),tht(id));
    PY = out_y;
    
%     [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o_h(i),o_px(i),o_py(i));
%     [AG_3,BG_3,CG_3,DG_3] = rectangle_plot(l_3,b_3,o3_h(i),o3_px(i),o3_py(i));
%     [AG_4,BG_4,CG_4,DG_4] = rectangle_plot(l_4,b_4,o4_h(i),o4_px(i),o4_py(i));
%     [AG_5,BG_5,CG_5,DG_5] = rectangle_plot(l_5,b_5,o5_h(i),o5_px(i),o5_py(i));
%     [AG_7,BG_7,CG_7,DG_7] = rectangle_plot(l_7,b_7,o7_h(i),o7_px(i),o7_py(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,r_h(i),PX,PY);
%     rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    rect2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
%     rect3 = [AG_3;BG_3;CG_3;DG_3;AG_3];
%     rect4 = [AG_4;BG_4;CG_4;DG_4;AG_4];
%     rect5 = [AG_5;BG_5;CG_5;DG_5;AG_5];
%     rect7 = [AG_7;BG_7;CG_7;DG_7;AG_7];
    
%     cent_1 = (AG_1 + CG_1)/2;
%     cent_3 = (AG_3 + CG_3)/2;
%     cent_4 = (AG_4 + CG_4)/2;
%     cent_5 = (AG_5 + CG_5)/2;
%     cent_7 = (AG_7 + CG_7)/2;
    
    %Check if circumcircle of obs and robo intersect for points of interest
    %and dist between them is decreasing ie coming closer
%     if((sqrt(power((cent_1(1) - r_cent(1)),2) + power((cent_1(2) - r_cent(2)),2)) - rad_1 - r_rad) < 0.6*i ...
%             && dGrad(1) <= 0)
%         flag(1) = 1;
%     end
%     if((sqrt(power((cent_3(1) - r_cent(1)),2) + power((cent_3(2) - r_cent(2)),2)) - rad_3 - r_rad) < 0.6*i ...
%             && dGrad(3) <= 0)
%         flag(3) = 1;
%     end
%     if((sqrt(power((cent_4(1) - r_cent(1)),2) + power((cent_4(2) - r_cent(2)),2)) - rad_4 - r_rad) < 0.6*i ...
%             && dGrad(4) <= 0)
%         flag(4) = 1;
%     end

    if(dGrad(1) <= 0)
        flag(1) = 1;
        sl_1 = (10/(sqrt(power((r_prev(1) - o_px(1)),2) + power((r_prev(2) - o_py(1)),2))))*l_1;
    else
        sl_1 = l_1;
    end  
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(sl_1,b_1,o_h(i),o_px(i),o_py(i));
    rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];

    if(dGrad(3) <= 0)
        flag(3) = 1;
        sl_3 = (10/(sqrt(power((r_prev(1) - o3_px(1)),2) + power((r_prev(2) - o3_py(1)),2))))*l_3;
    else
        sl_3 = l_3;
    end  
    [AG_3,BG_3,CG_3,DG_3] = rectangle_plot(sl_3,b_3,o3_h(i),o3_px(i),o3_py(i));
    rect3 = [AG_3;BG_3;CG_3;DG_3;AG_3];
    
    if(dGrad(4) <= 0)
        flag(4) = 1;
        sl_4 = (10*sqrt(2)/(sqrt(power((r_prev(1) - o4_px(1)),2) + power((r_prev(2) - o4_py(1)),2))))*l_4;
    else
        sl_4 = l_4;
    end   
    [AG_4,BG_4,CG_4,DG_4] = rectangle_plot(sl_4,b_4,o4_h(i),o4_px(i),o4_py(i));
    rect4 = [AG_4;BG_4;CG_4;DG_4;AG_4];

    if(dGrad(5) <= 0)
        flag(5) = 1;
        sl_5 = (5*sqrt(2)/(sqrt(power((r_prev(1) - o5_px(1)),2) + power((r_prev(2) - o5_py(1)),2))))*l_5;
    else
        sl_5 = l_5;
    end  
    [AG_5,BG_5,CG_5,DG_5] = rectangle_plot(sl_5,b_5,o5_h(i),o5_px(i),o5_py(i));
    rect5 = [AG_5;BG_5;CG_5;DG_5;AG_5];

    if(dGrad(7) <= 0)
        flag(7) = 1;
        sl_7 = (5/(sqrt(power((r_prev(1) - o7_px(1)),2) + power((r_prev(2) - o7_py(1)),2))))*l_7;
    else
        sl_7 = l_7;
    end  
    [AG_7,BG_7,CG_7,DG_7] = rectangle_plot(sl_7,b_7,o7_h(i),o7_px(i),o7_py(i));
    rect7 = [AG_7;BG_7;CG_7;DG_7;AG_7];
    
    rad_1 = sqrt(power(sl_1,2)+power(b_1,2))/2;
    rad_3 = sqrt(power(sl_3,2)+power(b_3,2))/2;
    rad_4 = sqrt(power(sl_4,2)+power(b_4,2))/2;
    rad_5 = sqrt(power(sl_5,2)+power(b_5,2))/2;
    rad_7 = sqrt(power(sl_7,2)+power(b_7,2))/2;
    
    olp1 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs1, robo
    olp3 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs3, robo
    olp4 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs4, robo
    olp5 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs5, robo
    olp7 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs7, robo
    
    %for iteration=1 we would like to solve for each obstacle and robot pair 
    p1 = 1;
    %Produce projects about axis of obstacle 1
    if(iter == 1 || (flag(1) ==1 && ...
      (sqrt(power((o_px(i) - r_prev(1)),2) + power((o_py(i) - r_prev(2)),2)) - rad_1 - r_rad) < 0.6*i))
    for j=2:3
        x = rect1(j,1) - rect1(j-1,1);
        y = rect1(j,2) - rect1(j-1,2);
        %
        x_ref = rect1(j-1,1);
        y_ref = rect1(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        disp(l);
        [olp1(p1), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                            checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l);
        fprintf('At p1 = %d',p1);
        %disp(olp1(p1));
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
%         w = vertcat(w{:});
%         g = vertcat(g{:});
        p1 = p1 + 1;
        l = l + 1;
    end
    end
% %             g = {g{:} olp1(1)*olp1(2)};
% %         lbg = [lbg; -inf];
% %         ubg = [ubg; 0];
    p3 = 1;
    %Produce projects about axis of obstacle 3
    if(iter == 1 || (flag(3) ==1 && ...
      (sqrt(power((o3_px(i) - r_prev(1)),2) + power((o3_py(i) - r_prev(2)),2)) - rad_3 - r_rad) < 0.6*i))
    for j=2:3
        x = rect3(j,1) - rect3(j-1,1);
        y = rect3(j,2) - rect3(j-1,2);
        %
        x_ref = rect3(j-1,1);
        y_ref = rect3(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        disp(l);
        [olp3(p3), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                            checkOLP_maxmin(rect3, rect2, x_rot, y_rot,x_ref,y_ref, ns, p3, l);
        fprintf('At p3 = %d',p3);
        %disp(olp3(p3));
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
%         w = vertcat(w{:});
%         g = vertcat(g{:});
        p3 = p3 + 1;
        l = l + 1;
    end
    end

    p4 = 1;
    %Produce projects about axis of obstacle 4
    if(iter == 1 || (flag(4) == 1 && ...
      (sqrt(power((o4_px(i) - r_prev(1)),2) + power((o4_py(i) - r_prev(2)),2)) - rad_4 - r_rad) < 0.6*i))
    for j=2:3
        x = rect4(j,1) - rect4(j-1,1);
        y = rect4(j,2) - rect4(j-1,2);
        %
        x_ref = rect4(j-1,1);
        y_ref = rect4(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        disp(l);
        [olp4(p4), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                            checkOLP_maxmin(rect4, rect2, x_rot, y_rot,x_ref,y_ref, ns, p4, l);
        fprintf('At p4 = %d',p4);
        %disp(olp1(p1));
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
%         w = vertcat(w{:});
%         g = vertcat(g{:});
        p4 = p4 + 1;
        l = l + 1;
    end
    end
    
    p5 = 1;
    %Produce projects about axis of obstacle 5
    if(iter == 1 || (flag(5) ==1 && ...
      (sqrt(power((o5_px(i) - r_prev(1)),2) + power((o5_py(i) - r_prev(2)),2)) - rad_5 - r_rad) < 0.6*i))
    for j=2:3
        x = rect5(j,1) - rect5(j-1,1);
        y = rect5(j,2) - rect5(j-1,2);
        %
        x_ref = rect5(j-1,1);
        y_ref = rect5(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        disp(l);
        [olp5(p5), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                            checkOLP_maxmin(rect5, rect2, x_rot, y_rot,x_ref,y_ref, ns, p5, l);
        fprintf('At p5 = %d',p5);
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];

        p5 = p5 + 1;
        l = l + 1;
    end
    end

    p7 = 1;
    %Produce projects about axis of obstacle 7
    if(iter == 1 || (flag(7) ==1 && ...
      (sqrt(power((o7_px(i) - r_prev(1)),2) + power((o7_py(i) - r_prev(2)),2)) - rad_7 - r_rad) < 0.6*i))
    for j=2:3
        x = rect7(j,1) - rect7(j-1,1);
        y = rect7(j,2) - rect7(j-1,2);
        %
        x_ref = rect7(j-1,1);
        y_ref = rect7(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        disp(l);
        [olp7(p7), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                            checkOLP_maxmin(rect7, rect2, x_rot, y_rot,x_ref,y_ref, ns, p7, l);
        fprintf('At p7 = %d',p7);
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];

        p7 = p7 + 1;
        l = l + 1;
    end
    end

    disp('I m out')
    %Produce projects about axis of robot
    for j=2:3
        x = rect2(j,1) - rect2(j-1,1);
        y = rect2(j,2) - rect2(j-1,2);
        %
        x_ref = rect2(j-1,1);
        y_ref = rect2(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
                
        fprintf('At l = %d',l);
        
%         [olp1(p1), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
%             checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l);
%         % Concatenate decision variables and constraint terms
%         w = {w{:} w_r{:}};
%         lbw = [lbw; lbw_r];
%         ubw = [ubw; ubw_r];
%         w0 = [w0; w0_r];
%         discrete = [discrete; discrete_r];
%         g = {g{:} g_r{:}};
%         lbg = [lbg; lbg_r];
%         ubg = [ubg; ubg_r];
%         p1 = p1 + 1;
%         l = l + 1;

        if(iter == 1 || (flag(1) ==1 && ...
      (sqrt(power((o_px(i) - r_prev(1)),2) + power((o_py(i) - r_prev(2)),2)) - rad_1 - r_rad) < 0.6*i))
        [olp1(p1), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l);
        
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        p1 = p1 + 1;
        l = l + 1;
        end

        if(iter == 1 || (flag(3) ==1&& ...
      (sqrt(power((o3_px(i) - r_prev(1)),2) + power((o3_py(i) - r_prev(2)),2)) - rad_3 - r_rad) < 0.6*i))
        [olp3(p3), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            checkOLP_maxmin(rect3, rect2, x_rot, y_rot,x_ref,y_ref, ns, p3, l);
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        p3 = p3 + 1;
        l = l + 1;
        end
        
        if(iter == 1 || (flag(4) ==1 && ...
      (sqrt(power((o4_px(i) - r_prev(1)),2) + power((o4_py(i) - r_prev(2)),2)) - rad_4 - r_rad) < 0.6*i))
        [olp4(p4), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            checkOLP_maxmin(rect4, rect2, x_rot, y_rot,x_ref,y_ref, ns, p4, l);
        
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        p4 = p4 + 1;
        l = l + 1;
        end

        if(iter == 1 || (flag(5) ==1&& ...
      (sqrt(power((o5_px(i) - r_prev(1)),2) + power((o5_py(i) - r_prev(2)),2)) - rad_5 - r_rad) < 0.6*i))
        [olp5(p5), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            checkOLP_maxmin(rect5, rect2, x_rot, y_rot,x_ref,y_ref, ns, p5, l);
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        p5 = p5 + 1;
        l = l + 1;
        end
        
        if(iter == 1 || (flag(7) ==1&& ...
      (sqrt(power((o7_px(i) - r_prev(1)),2) + power((o7_py(i) - r_prev(2)),2)) - rad_7 - r_rad) < 0.6*i))
        [olp7(p7), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            checkOLP_maxmin(rect7, rect2, x_rot, y_rot,x_ref,y_ref, ns, p7, l);
        
        % Concatenate decision variables and constraint terms
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        p7 = p7 + 1;
        l = l + 1;
        end
    end

%     J = J + R*olp1(1)*olp1(2)*olp1(3)*olp1(4);%*olp(5)*olp(6)*olp(7)*olp(8); 
     J = J + R*olp1(1)*olp1(2)*olp1(3)*olp1(4) + R*olp3(1)*olp3(2)*olp3(3)*olp3(4) ...
        + R*olp4(1)*olp4(2)*olp4(3)*olp4(4) + R*olp5(1)*olp5(2)*olp5(3)*olp5(4) + R*olp7(1)*olp7(2)*olp7(3)*olp7(4);

%    J = J + R*olp4(1)*olp4(2)*olp4(3)*olp4(4);% + R*olp3(1)*olp3(2)*olp3(3)*olp3(4);% + R*olp1(1)*olp1(2)*olp1(3)*olp1(4);
%     J = J + Q*((PX(1) - r_px(i))^2  + (PY(1) - r_py(i))^2);
    J = J + Q*((PX(1) - g_2(1))^2  + (PY(1) - g_2(2))^2);
%     end
    if(i>1)
        J = J + 0.1*(tht(id)-tht(id-1));
    end
%     if(i == N-1)
%         J = J + Q*((PX(1) - g_2(1))^2  + (PY(1) - g_2(2))^2);
%     end

id = id + 1;
end
 w = vertcat(w{:});
 g = vertcat(g{:});
 %      w = w{:};
 lbw = lbw(:);
 ubw = ubw(:);
 w0 = w0(:);
 discrete = discrete(:);
 %     g = g{:};
 lbg = lbg(:);
 ubg = ubg(:);
 disp('Here are the lengths')
 length(w)
 length(discrete)
 
% Create an NLP solver
nlp_prob = struct('f', J, 'x', w, 'g', g);
nlp_solver = nlpsol('nlp_solver', 'bonmin', nlp_prob, struct('discrete', discrete));
% nlp_solver = nlpsol('nlp_solver', 'ipopt', nlp_prob);
% Solve the NLP
sol = nlp_solver('x0',w0, 'lbx',lbw, 'ubx',ubw, 'lbg',lbg, 'ubg',ubg);
w_opt = full(sol.x);

% Compute state trajectory integrating the dynamics
x_opt       = zeros(nx,nv+1);
x_opt(1)    = x_0;
y_opt       = zeros(nx,nv+1);
y_opt(1)    = y_0;
for i=2:nv+1
    out_x       = RK4x(x_opt(i-1),w_opt(i-1),w_opt(nv+i-1));
    x_opt(i)    = full(out_x);
    
    out_y       = RK4y(y_opt(i-1),w_opt(i-1),w_opt(nv+i-1));
    y_opt(i)    = full(out_y);
end

x_out = x_opt;
y_out = y_opt;
v_out = w_opt(1:nv);
th_out = w_opt(nv+1:2*nv);
% d_values = w_opt(end-24+1:end);
%d_values = [sl_4];
d_values = [sl_1,sl_3,sl_4,sl_5,sl_7];
%% Plot results

% figure()
% subplot(2,1,1)
% plot((0:N)*T/N, x_opt)
% xlabel('time - t')
% ylabel('state - x')
% grid on
% subplot(2,1,2)
% stairs((0:N-1)*T/N, u_opt(1:N),'r')
% xlabel('time - t')
% ylabel('input - u')
% grid on
% 
% figure()
% subplot(2,1,1)
% plot((0:N)*T/N, y_opt)
% xlabel('time - t')
% ylabel('state - y')
% grid on
% subplot(2,1,2)
% stairs((0:N-1)*T/N, u_opt(N+1:end),'r')
% xlabel('time - t')
% ylabel('input - u')
% % grid on
% 
% figure()
% plot(x_opt,y_opt)
