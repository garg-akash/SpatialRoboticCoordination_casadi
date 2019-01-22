function [v_out, th_out, x_out, y_out, d_values] = oru_sptl_OC_m2_3obs(N, del_t, r_st, o_px, o_py, o_h, r_px, r_py, r_h ...
                                           , o3_px, o3_py, o3_h, l_1, b_1, l_2, b_2, l_3, b_3)
% To implement loop optimal control
% Xunit_2 = x_opt;
% Yunit_2 = y_opt;
% Hfinal_2 = atan2(u_opt(41:end),u_opt(1:40));
import casadi.*
nx  = 1;
nu  = 1;
h = del_t; %discretization step
ns = 4; %number of sides in polygon

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
tht   = MX.sym('tht');
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
nv = N;
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
lbw = [lbw; -0.8*ones(N,1); -0.9*ones(N,1)];
ubw = [ubw; 0.8*ones(N,1); 0.9*ones(N,1)];
w0 = [w0; 0.7*ones(N,1); 0.7*ones(N,1)];
discrete = [discrete; zeros(N,1); zeros(N,1)];
l = 1; %It's a global variable to number the decision varibales
olp_all = SX.zeros(2*ns*N,1);  %keep overlap value
for i = 1:N
    out_x = RK4x(PX,V(i),tht(i));
    PX = out_x;
    
    out_y = RK4y(PY,V(i),tht(i));
    PY = out_y;
    
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o_h(i),o_px(i),o_py(i));
    [AG_3,BG_3,CG_3,DG_3] = rectangle_plot(l_3,b_3,o3_h(i),o3_px(i),o3_py(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,0,PX,PY);
    rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    rect2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    rect3 = [AG_3;BG_3;CG_3;DG_3;AG_3];
    %disp(rect2);

    olp1 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs1, robo
    olp3 = SX.zeros(2*ns,1);  %keep overlap value for just 1 orientation of obs3, robo
    
    p1 = 1;
    %Produce projects about axis of obstacle 1
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
    
    p3 = 1;
    %Produce projects about axis of obstacle 3
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
        disp(l);
        [olp1(p1), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                        checkOLP_maxmin(rect1, rect2, x_rot, y_rot,x_ref,y_ref, ns, p1, l);
        [olp3(p3), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
                                        checkOLP_maxmin(rect3, rect2, x_rot, y_rot,x_ref,y_ref, ns, p3, l);
        
        fprintf('At p1 = %d',p1);
        fprintf('At p3 = %d',p3);
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
        p1 = p1 + 1;
        p3 = p3 + 1;
        l = l + 1;
    end
    
    if (i==N/3 || i==2*N/3)% || i==3*N/3 || i== N)
        J = J + Q*((PX(1) - r_px(i))^2  + (PY(1) - r_py(i))^2);
    elseif(i==N)
        J = J + (Q/2)*((PX(1) - r_px(i))^2  + (PY(1) - r_py(i))^2);
    end

    %J = J + R*olp(1)*olp(2)*olp(3)*olp(4)*olp(5)*olp(6)*olp(7)*olp(8); 
    J = J + R*olp1(1)*olp1(2) + R*olp3(1)*olp3(2);
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
 length(w)
 length(discrete)
 
% Create an NLP solver
nlp_prob = struct('f', J, 'x', w, 'g', g);
nlp_solver = nlpsol('nlp_solver', 'bonmin', nlp_prob, struct('discrete', discrete));

% Solve the NLP
sol = nlp_solver('x0',w0, 'lbx',lbw, 'ubx',ubw, 'lbg',lbg, 'ubg',ubg);
w_opt = full(sol.x);

% Compute state trajectory integrating the dynamics
x_opt       = zeros(nx,N+1);
x_opt(1)    = x_0;
y_opt       = zeros(nx,N+1);
y_opt(1)    = y_0;
for i=2:N+1
    out_x       = RK4x(x_opt(i-1),w_opt(i-1),w_opt(N+i-1));
    x_opt(i)    = full(out_x);
    
    out_y       = RK4y(y_opt(i-1),w_opt(i-1),w_opt(N+i-1));
    y_opt(i)    = full(out_y);
end

x_out = x_opt;
y_out = y_opt;
v_out = w_opt(1:N);
th_out = w_opt(N+1:2*N);
d_values = w_opt(end-24+1:end);
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
