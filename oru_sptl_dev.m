function [v_out, omg_out, p_out, t_out, x_out, y_out] = oru_sptl_dev(N, del_t, r_st, o1_px, o1_py, o1_h, r_px, r_py, r_h ...
                                , l_1, b_1, l_2, b_2, nv, flag1, ind_ob, ind_left, sim_no, vel_0, omg_0)

import casadi.*
nx  = 4;
nu  = 2;
h = del_t; %discretization step
ns = 4; %number of sides in polygon
L = 1.19; %rear to front distance
d_rear = L/2; %rear center to body center distance
Q = 1;
R = 0.5;
% S = 1;

obj.states = MX.sym('Sxy',ns,1);             %[x,y,theta,phi]
obj.controls = MX.sym('Uxy',nu,1);          %[v, omega ]
S = obj.states;
U = obj.controls;
f_expr = [U(1)*cos(S(3))       ;...       %x_dot = V cos(theta)
         U(1)*sin(S(3))        ;...       %y_dot = V sin(theta)
         U(1)*tan(S(4))/L      ;...       %theta_dot = V*tan(phi)/L
         U(2)                 ];     %phi_dot = omega (angular_velocity) 

f = Function('f',{S,U}, {f_expr});

s0 = MX.sym('s0',ns);
s = s0;
u = MX.sym('u',nu);

k1 = f(s,u);
k2 = f(s+0.5*h*k1,u);
k3 = f(s+0.5*h*k2,u);
k4 = f(s+h*k3,u);
s = s + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

F = Function('F',{s0,u},{s});

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

s = SX.sym('x', ns, nv+1);
s(:,1) = r_st;

l = 1; %It's a global variable to number the decision variables
for i = 1:N/nv:N
    
    u = SX.sym(['u_' num2str(i)],nu); 
    w = {w{:} u};
    lbw = [lbw; -1; -1];
    ubw = [ubw; 1; 1];
    w0 = [w0; vel_0(i); omg_0(i)];
    discrete = [discrete; 0; 0];
    res = F(s(:,i),u); %integrator
    s(:,i+1) = res(1:nx);
    J = J + Q*((res(1) - r_px(i))^2  + (res(2) - r_py(i))^2);
    if((flag1(i) == 1) && (ind_left>0))
        [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o1_h(ind_ob),o1_px(ind_ob),o1_py(ind_ob));
        rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
        [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,r_h(i),res(1) + d_rear*cos(r_h(i)),res(2) + d_rear*sin(r_h(i)));
        rect2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
        
        olp1 = SX.zeros(ns,1);
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
            p1 = p1 + 1;
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
            
            fprintf('At l = %d',l);
            
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
        ind_left = ind_left - N/nv;
        ind_ob = ind_ob + N/nv;
        J = J + R*olp1(1)*olp1(2)*olp1(3)*olp1(4);
    end
end
w = vertcat(w{:});
g = vertcat(g{:});
lbw = lbw(:);
ubw = ubw(:);
w0 = w0(:);
discrete = discrete(:);
lbg = lbg(:);
ubg = ubg(:);
disp('Here are the lengths')
length(w)
length(discrete)

% Create an NLP solver
nlp_prob = struct('f', J, 'x', w, 'g', g);
nlp_solver = nlpsol('nlp_solver', 'bonmin', nlp_prob, struct('discrete', discrete));
%  nlp_solver = nlpsol('nlp_solver', 'ipopt', nlp_prob);
% Solve the NLP
sol = nlp_solver('x0',w0, 'lbx',lbw, 'ubx',ubw, 'lbg',lbg, 'ubg',ubg);
w_opt = full(sol.x);

% Compute state trajectory integrating the dynamics
x_opt       = zeros(nx,nv+1);
x_opt(:,1)    = r_st;

v_opt = w_opt(1:nu:end);
omg_opt = w_opt(2:nu:end);

for i=2:nv+1
    x_opt(:,i) = full(F(x_opt(:,i-1),[v_opt(i-1);omg_opt(i-1)]));
end
p_out = x_opt(4:nx:end,1:nv+1);
t_out = x_opt(3:nx:end,1:nv+1);
x_out = x_opt(1:nx:end,1:nv+1);
y_out = x_opt(2:nx:end,1:nv+1);
v_out = v_opt;
omg_out = omg_opt;
