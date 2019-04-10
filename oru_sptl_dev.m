function [v_out, omg_out, p_out, t_out, x_out, y_out, d_out] = oru_sptl_dev(N, throw, del_t, r_st, o1_px, o1_py, o1_h, r_px, r_py, r_h ...
                                , l_1, b_1, l_2, b_2, nv, flag1, ind_ob, ind_left, c_sim, t_sim ...
                                , A0, A1, b, th, vel_0, omg_0, goal, omg_pichla, vel_pichla, flag2)
import casadi.*
ind_ob_v = ind_ob;
ind_left_v = ind_left;
nx  = 4;
nu  = 2;
h = del_t; %discretization step
ns = 4; %number of sides in polygon
L = 1.19; %rear to front distance
d_rear = L/2; %rear center to body center distance
%Q = 0.5;
Q = 0.5 + (1-0.5)*(c_sim-1)/(t_sim-1)
%R = 10;
R = 15 - 15*(c_sim-1)/(t_sim-1)
%T = 0.5;
T = 0.5 - 0.5*(c_sim-1)/(t_sim-1)
W = 1.5;

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

% dae = struct('x',S,'p',U,'ode',f_expr);
% opts = struct('tf',0.5);
% F = integrator('F', 'cvodes', dae,opts);

% Fk = F('x0',[0.2; 0.3; 0.1; 0.002],'p',[0.5; 0.2]);
% disp(Fk.xf)

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

s = SX.sym('s', nx, N+1);
s(:,1) = r_st;

l = 1; %It's a global variable to number the decision variables
id = 1;
track_u = [];
for i = 1:N
    
    u = SX.sym(['u_' num2str(i)],nu); 
    w = {w{:} u};
    lbw = [lbw; 0; -1];
    ubw = [ubw; 1; 1];
    w0 = [w0; vel_0(i); omg_0(i)];
%     w0 = [w0; 0.5;0.5];
    discrete = [discrete; 0; 0];
    g = {g{:} u(1)-vel_pichla};
    lbg = [lbg; -0.5];
    ubg = [ubg; 0.5];

    track_u = [track_u;ones(nu,1)];
%     Fk = F('x0',s(:,id),'p',u); %integrator
%     res = Fk.xf;
    res = F(s(:,id),u);
    s(:,id+1) = res(1:nx);
%     if((t_sim - c_sim) <  2*(N - throw))
%         Q = 1;
%         R = 0;
%         T = 0.1;
%     end
%     J = J + Q*((res(1) - r_px(i))^2  + (res(2) - r_py(i))^2);
    J = J + Q*((res(1) - goal(1))^2  + (res(2) - goal(2))^2);
%     if(c_sim >= 0.6*t_sim)
%         J = J + Q*(res(3) - r_h(i));
%     end
%     if ((flag2<0))
%         R = R/10;
%         T = T/2;
%         J = J + W*((res(1) - goal(1))^2  + (res(2) - goal(2))^2);
%         %     g = {g{:} res(1) res(2)};
%         %     lbg = [lbg; goal(1); r_py(i)];
%         %     ubg = [ubg; r_px(i); r_py(i)];
%     end
%     if(i==(N - throw) && ((t_sim - c_sim) <= (N - throw)))
%         disp('Entered!!!!!!!!!!!!!!')
%         g = {g{:} res(1) res(2) res(3)};
%         lbg = [lbg; goal(1); goal(2); goal(3) + deg2rad(-5)];
%         ubg = [ubg; goal(1); goal(2); goal(3) + deg2rad(5)];
%     end
%     if(i==N)
%         disp('Entered!!!!!!!!!!!!!!')
%         r_px(i)
%         r_py(i)
%         g = {g{:} res(1) res(2)};
%         lbg = [lbg; r_px(i); r_py(i)];
%         ubg = [ubg; r_px(i); r_py(i)];
%     end
    J = J + T*(u(1)^2) + T*(u(2)^2);
    if(ismember(i,1:N/nv:N))
    if((flag1(i) == 1) && (ind_left>0))
        [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o1_h(ind_ob),o1_px(ind_ob) + d_rear*cos(o1_h(ind_ob)),o1_py(ind_ob) + d_rear*sin(o1_h(ind_ob)));
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
            
            fprintf('At l = %d',l);
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
            track_u = [track_u;zeros(length(w_r),1)];
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
            track_u = [track_u;zeros(length(w_r),1)];
            p1 = p1 + 1;
            l = l + 1;
        end
        ind_left = ind_left - N/nv;
        ind_ob = ind_ob + N/nv;
%         cost_2 = (olp1(1)*olp1(2)*olp1(3)*olp1(4));
        cost_2 = min(olp1);
        J = J + R*cost_2;
    end
    end
     
    for indx = 1:4
        g = {g{:} A0(i,indx)*res(1) + A1(i,indx)*res(2)};
        lbg = [lbg; -inf];
        ubg = [ubg; b(i,indx)];
    end
    
    g = {g{:} res(3)};
    lbg = [lbg; th(i,1)];
    ubg = [ubg; th(i,2)];
    
%     if ((t_sim - c_sim) > (N - throw))
%         m = SX.sym(['m_' num2str(i)],N); 
%         w = {w{:} m};
%         lbw = [lbw; zeros(N,1)];
%         ubw = [ubw; ones(N,1)];
%         w0 = [w0; zeros(N,1)];
%         discrete = [discrete; ones(N,1)];
%         g = {g{:} g_r{:}};
%         lbg = [lbg; lbg_r];
%         ubg = [ubg; ubg_r];
%     end
    
vel_pichla = u(1);
id = id + 1;
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
x_opt       = zeros(nx,N+1);
x_opt(:,1)    = r_st;

u_opt = w_opt(find(track_u));
track_d = ~track_u;
d_out = w_opt(find(track_d));
v_opt = u_opt(1:2:end);
omg_opt = u_opt(2:2:end);

for i=2:N+1
    x_opt(:,i) = full(F(x_opt(:,i-1),[v_opt(i-1);omg_opt(i-1)]));
end

p_out = x_opt(4:nx:end,1:N+1);
t_out = x_opt(3:nx:end,1:N+1);
x_out = x_opt(1:nx:end,1:N+1);
y_out = x_opt(2:nx:end,1:N+1);
v_out = v_opt;
omg_out = omg_opt;

% %TO verify the overlap stuff
% l_v = 1;
% overlap_v = [];
% overlap_a = [];
% d1_v = d_out(1:4:end);
% d2_v = d_out(2:4:end);
% d3_v = d_out(3:4:end);
% d4_v = d_out(4:4:end);
% for i=1:N/nv:N
%     if((flag1(i) == 1) && (ind_left_v>0))
%         [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o1_h(ind_ob_v),o1_px(ind_ob_v) + d_rear*cos(o1_h(ind_ob_v)),o1_py(ind_ob_v) + d_rear*sin(o1_h(ind_ob_v)));
%         rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
%         [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,r_h(i),x_out(i+1) + d_rear*cos(r_h(i)),y_out(i+1) + d_rear*sin(r_h(i)));
%         rect2 = [AG_2;BG_2;CG_2;DG_2;AG_2];    
%         Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
%         Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
%         [x_int,y_int] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
%         overlap_a = [overlap_a;polyarea(x_int,y_int)];
%         olp1_v = zeros(ns,1);
%         p1_v = 1;
%         %Produce projects about axis of obstacle 1
%         for j=2:3
%             x = rect1(j,1) - rect1(j-1,1);
%             y = rect1(j,2) - rect1(j-1,2);
%             %
%             x_ref = rect1(j-1,1);
%             y_ref = rect1(j-1,2);
%             
%             % compute the perpendcular to the edge vector
%             x_rot = -y;
%             y_rot = x;
%             side1 = zeros(1,ns);
%             side2 = zeros(1,ns);
%             for k=1:4
%                 side1(k) = x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref);
%             end
%             for k=1:4
%                 side2(k) = x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref);
%             end
%             
%             mx_s1 = max(side1);
%             mn_s1 = min(side1);
%             
%             mx_s2 = max(side2);
%             mn_s2 = min(side2);
%             olp1_v(p1_v) = (((mx_s2 - mn_s2)*(1-d4_v(l_v)) + (mx_s2-mn_s1)*d4_v(l_v))*d3_v(l_v) ...
%                     + ((mx_s1 - mn_s2)*(1-d4_v(l_v)) + (mx_s1-mn_s1)*d4_v(l_v))*(1-d3_v(l_v)))*(1-d1_v(l_v))*(1-d2_v(l_v));
%             l_v = l_v + 1;
%             p1_v = p1_v + 1;
%         end
%         for j=2:3
%             x = rect2(j,1) - rect2(j-1,1);
%             y = rect2(j,2) - rect2(j-1,2);
%             %
%             x_ref = rect2(j-1,1);
%             y_ref = rect2(j-1,2);
%             
%             % compute the perpendcular to the edge vector
%             x_rot = -y;
%             y_rot = x;
%             side1 = zeros(1,ns);
%             side2 = zeros(1,ns);
%             for k=1:4
%                 side1(k) = x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref)/sqrt(x_rot^2 + y_rot^2);;
%             end
%             for k=1:4
%                 side2(k) = x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref)/sqrt(x_rot^2 + y_rot^2);;
%             end
%             
%             mx_s1 = max(side1);
%             mn_s1 = min(side1);
%             
%             mx_s2 = max(side2);
%             mn_s2 = min(side2);
%             olp1_v(p1_v) = (((mx_s2 - mn_s2)*(1-d4_v(l_v)) + (mx_s2-mn_s1)*d4_v(l_v))*d3_v(l_v) ...
%                     + ((mx_s1 - mn_s2)*(1-d4_v(l_v)) + (mx_s1-mn_s1)*d4_v(l_v))*(1-d3_v(l_v)))*(1-d1_v(l_v))*(1-d2_v(l_v));
%             l_v = l_v + 1;
%             p1_v = p1_v + 1;
%         end
%         olp1_v
%         ind_left_v = ind_left_v - N/nv;
%         ind_ob_v = ind_ob_v + N/nv;
%         overlap_v = [overlap_v;(olp1_v(1)*olp1_v(2)*olp1_v(3)*olp1_v(4))];
%     end
% end
% disp('Min of olp')
% min(olp1_v)
% disp('Calculated overlap')
% overlap_v
% disp('Actual overlap')
% overlap_a