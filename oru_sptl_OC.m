function [vx_out, vy_out, x_out, y_out] = oru_sptl_OC(N, del_t, r_st, o_px, o_py, o_h, r_px, r_py, r_h ...
                                           , l_1, b_1, l_2, b_2, ar)
% To implement loop optimal control
% Xunit_2 = x_opt;
% Yunit_2 = y_opt;
% Hfinal_2 = atan2(u_opt(41:end),u_opt(1:40));
import casadi.*
nx  = 1;
nu  = 1;
h = del_t; %discretization step

Q = 0.5;
R = 0.5;

Vx = SX.sym('Vx',nu);
Vy = SX.sym('Vy',nu);
Px = SX.sym('Px',nx);
Py = SX.sym('Py',nx);

xdot = Vx(1);
ydot = Vy(1);

f = Function('f', {Px,Vx}, {xdot});

% RK4
VX   = MX.sym('VX');
PX0  = MX.sym('PX0',nx);
PX   = PX0;

k = f(PX,VX);
k1 = k;
k = f(PX + h/2 * k1, VX);
k2 = k;
k = f(PX + h/2 * k2, VX);
k3 = k;
k = f(PX + h   * k3, VX);
k4 = k;
PX = PX + h/6*(k1   + 2*k2   + 2*k3   + k4);
RK4 = Function('RK4',{PX0,VX},{PX});

% Formulate NLP (use matrix graph)
nv = N;
Vx = SX.sym('Vx',nv);
Vy = SX.sym('Vy',nv);

% Objective function
J=0;

% Get an expression for the cost and state at end
x_0 = r_st(1); %starting position 
PX = x_0;
y_0 = r_st(2); 
PY = y_0;
for i = 1:N
    out_x = RK4(PX,Vx(i));
    PX = out_x;
    
    out_y = RK4(PY,Vy(i));
    PY = out_y;
    
    %J = J + Q*PX(1)^2 + R*Vx(i)^2;
%     [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,o_h(i),o_px(i),o_py(i));
%     [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,r_h(i),r_px(i),r_py(i));
%     Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
%     Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    
    wt = (PY(1) - r_py(i))/(o_py(i) - r_py(i));
    
    %wt = (PX(1) - r_px(i))/(o_px(i) - r_px(i));
    %wt = ((PX(1) - r_px(i))*(PY(1) - r_py(i)))/(o_py(i) - r_py(i));
    %wt = (PX(1) - o_px(i))^2 + (PY(1) - o_py(i))^2;
    
    %steer = atan2(Vy(i),Vx(i));
    
    if (i==N/4 || i==2*N/4 || i==3*N/4 || i== N)
        J = J + Q*((PX(1) - r_px(i))^2  + (PY(1) - r_py(i))^2);
    end
    
    J = J + R*wt*ar(i);  %+ 10*steer;
end

% Terminal constraints: x_0(T)=x_1(T)=0
% Here PX PY are the terminal Robot Positions 
g = [atan2(Vy(2:end),Vx(2:end))-atan2(Vy(1:end-1),Vx(1:end-1))];

% Allocate an NLP solver
nlp = struct('x', [Vx;Vy], 'f', J, 'g', g);

% Create IPOPT solver object
options = struct;
options.hessian_approximation = 'limited-memory';
solver = nlpsol('solver', 'ipopt', nlp);

% Solve the problem
% Don't keep x0=0, atan2(Vy(1),Vx(1)) will give inf 
res = solver('x0',1,'lbx',[-0.5],'ubx',[0.5],'lbg',[-0.8*ones(N-1,1)],'ubg',[0.8*ones(N-1,1)]);

f_opt       = full(res.f);
u_opt       = full(res.x);

% Compute state trajectory integrating the dynamics
x_opt       = zeros(nx,N+1);
x_opt(1)    = x_0;
y_opt       = zeros(nx,N+1);
y_opt(1)    = y_0;
for i=2:N+1
    out_x       = RK4(x_opt(i-1),u_opt(i-1));
    x_opt(i)    = full(out_x);
    
    out_y = RK4(y_opt(i-1),u_opt(N + i-1));
    y_opt(i) = full(out_y);
end

x_out = x_opt;
y_out = y_opt;
vx_out = u_opt(1:N);
vy_out = u_opt(N+1:end);
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
