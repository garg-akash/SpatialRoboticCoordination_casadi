function [v_out, omg_out, p_out, t_out, x_out, y_out] = oru_sptl_dev(N, del_t, r_st, o1_px, o1_py, o1_h, r_px, r_py, r_h ...
                                , l_1, b_1, l_2, b_2, nv, flag1, ind_ob, ind_left, sim_no, v_prev, omg_prev)
import casadi.*
nx  = 1;
nu  = 1;
h = del_t; %discretization step
ns = 4; %number of sides in polygon
L = 1; %rear to front distance
d_rear = L/2;
Q = 0.4;
R = 0.6;
S = 0.1;
V = SX.sym('V',nu);
tht = SX.sym('tht',nu);
phi = SX.sym('phi',nu);
omg = SX.sym('omg',nu);
Px = SX.sym('Px',nx);
Py = SX.sym('Py',nx);

xdot = V(1)*cos(tht(1));
ydot = V(1)*sin(tht(1));
tdot = (V(1)/L)*tan(phi(1));
pdot = omg(1);
% Intergrator for x and y
fy = Function('fy', {Py,V,tht},{ydot});
fx = Function('fx', {Px,V,tht},{xdot});
% Integrator for phi
fp = Function('fp', {phi,omg},{pdot});
% Integrator for tht
ft = Function('ft', {tht,V,phi},{tdot});

V      = MX.sym('V');
tht    = MX.sym('tht');
phi    = MX.sym('phi');
omg    = MX.sym('omg');
% RK4
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

% RK4
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

% RK4
PT0  = MX.sym('PT0',nx);
PT   = PT0;

k = ft(PT,V,phi);
k1 = k;
k = ft(PT + h/2 * k1, V, phi);
k2 = k;
k = ft(PT + h/2 * k2, V, phi);
k3 = k;
k = ft(PT + h   * k3, V, phi);
k4 = k;
PT = PT + h/6*(k1   + 2*k2   + 2*k3   + k4);
RK4t = Function('RK4t',{PT0,V,phi},{PT});

% RK4
PP0  = MX.sym('PP0',nx);
PP   = PP0;

k = fp(PP,omg);
k1 = k;
k = fp(PP + h/2 * k1, omg);
k2 = k;
k = fp(PP + h/2 * k2, omg);
k3 = k;
k = fp(PP + h   * k3, omg);
k4 = k;
PP = PP + h/6*(k1   + 2*k2   + 2*k3   + k4);
RK4p = Function('RK4p',{PP0,omg},{PP});


V = SX.sym('V',nv);
%tht = SX.sym('tht',nv);
%phi = SX.sym('phi',nv);
omg = SX.sym('omg',nv);

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
t_0 = r_st(3);
PT = t_0;
p_0 = r_st(4);
PP = p_0;

w = {V omg};
% if(sim_no == 1)
    lbw = [lbw; 0*ones(nv,1); -0.5*ones(nv,1)];
    ubw = [ubw; 0.7*ones(nv,1); 0.5*ones(nv,1)];
    w0 = [w0; 0.6*ones(nv,1); 0.2*ones(nv,1)];
% else
%     %     lbw = [lbw; min(v_prev-0.2*ones(nv-1,1),0*ones(nv-1,1)); min(v_prev(end)-0.2,0.7); omg_prev-0.2*ones(nv-1,1); omg_prev(end)-0.2];
%     %     ubw = [ubw; min(v_prev+0.2*ones(nv-1,1),0.7*ones(nv-1,1)); min(v_prev(end)+0.2,0.7); omg_prev+0.2*ones(nv-1,1); omg_prev(end)+0.2];
%     lbw = [lbw; v_prev-0.2*ones(nv-1,1); v_prev(end)-0.2; omg_prev-0.2*ones(nv-1,1); omg_prev(end)-0.2];
%     ubw = [ubw; v_prev+0.2*ones(nv-1,1); v_prev(end)+0.2; omg_prev+0.2*ones(nv-1,1); omg_prev(end)+0.2];
%     w0 = [w0; v_prev; v_prev(end); omg_prev; omg_prev(end)];
% end
discrete = [discrete; zeros(nv,1); zeros(nv,1)];

l = 1; %It's a global variable to number the decision variables
id = 1;
for i = 1:N/nv:N
    
    out_phi = RK4p(PP,omg(id));
    PP = out_phi;
    
    out_t = RK4t(PT,V(id),PP);
    PT = out_t;
    
    out_x = RK4x(PX,V(id),PT);
    PX = out_x;
    
    out_y = RK4y(PY,V(id),PT);
    PY = out_y;
    if((flag1(i) == 1) && (ind_left>0))
        [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o1_h(ind_ob-1+i),o1_px(ind_ob-1+i),o1_py(ind_ob-1+i));
        rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
        [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,r_h(i),PX + d_rear*cos(r_h(i)),PY + d_rear*sin(r_h(i)));
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
        J = J + R*olp1(1)*olp1(2)*olp1(3)*olp1(4);
    end
    J = J + Q*((PX(1) - r_px(i))^2  + (PY(1) - r_py(i))^2);
    
    J = J + S*(omg(id)^2);
%     if(i>1)
%         J = J + 0.1*(tht(id)-tht(id-1));
%     end
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
% nlp_solver = nlpsol('nlp_solver', 'ipopt', nlp_prob);
% Solve the NLP
sol = nlp_solver('x0',w0, 'lbx',lbw, 'ubx',ubw, 'lbg',lbg, 'ubg',ubg);
w_opt = full(sol.x);

% Compute state trajectory integrating the dynamics
x_opt       = zeros(nx,nv+1);
x_opt(1)    = x_0;
y_opt       = zeros(nx,nv+1);
y_opt(1)    = y_0;
t_opt       = zeros(nx,nv+1);
t_opt(1)    = t_0;
p_opt       = zeros(nx,nv+1);
p_opt(1)    = p_0;
for i=2:nv+1  
    out_p       = RK4p(p_opt(i-1),w_opt(nv+i-1));
    p_opt(i)    = full(out_p);
        
    out_t       = RK4t(t_opt(i-1),w_opt(i-1),w_opt(nv+i-1));
    t_opt(i)    = full(out_t);
    
    out_x       = RK4x(x_opt(i-1),w_opt(i-1),t_opt(i));
    x_opt(i)    = full(out_x);
    
    out_y       = RK4y(y_opt(i-1),w_opt(i-1),t_opt(i));
    y_opt(i)    = full(out_y);
end
p_out = p_opt;
t_out = t_opt;
x_out = x_opt;
y_out = y_opt;
v_out = w_opt(1:nv);
omg_out = w_opt(nv+1:2*nv);