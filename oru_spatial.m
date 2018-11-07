
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')

import casadi.*
% Constants
th = pi:pi/39:2*pi;
N = length(th);
total_t = 40;
T = total_t;
del_t = total_t/(2*N);
h = del_t; %discretization step

c_1 = [0,3.5];
c_2 = [0,-3.5];
% c_1 = [-3,4];
% c_2 = [4,1];
r = 4;
avg_vel = 2*pi*r/total_t;

steps = total_t/del_t;

%%footprint of obstacle/unit 1
hinit_1 = 3*pi/2; % initial heading for unit 1
b_1 = 0.75;
l_1 = 1.5;

%%footprint of robot/unit 2
hinit_2 = pi/2; % initial heading for unit 2
b_2 = 0.75;
l_2 = 1.5;
Xunit_1=[];
Yunit_1=[];
Hfinal_1=[];
Xunit_2=[];
Yunit_2=[];
Hfinal_2=[];
Vxunit_2=[];
Vyunit_2=[];
xnow_1 = c_1(1)-r;
ynow_1 = c_1(2);
xnow_2 = c_2(1)-r;
ynow_2 = c_2(2);

for i=1:length(th)
    xunit_1 = xnow_1;
    yunit_1 = ynow_1;
    Xunit_1 = [Xunit_1;xunit_1];
    Yunit_1 = [Yunit_1;yunit_1];
    hfinal_1 = th(i) + hinit_1;
    Hfinal_1 = [Hfinal_1;hfinal_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2;xunit_2];
    Yunit_2 = [Yunit_2;yunit_2];
    hfinal_2 = th(i) + hinit_2;
    Hfinal_2 = [Hfinal_2;hfinal_2];
    
    xdel_1 = avg_vel*(-sin(th(i)))*del_t;
    ydel_1 = avg_vel*(cos(th(i)))*del_t;
    xnow_1 = xdel_1 + xunit_1;
    ynow_1 = ydel_1 + yunit_1;
    
    xdel_2 = -avg_vel*(-sin(2*pi-th(i)))*del_t; %compared to 1 direction of motion is opposite for 2
    ydel_2 = -avg_vel*(cos(2*pi-th(i)))*del_t;
    xnow_2 = xdel_2 + xunit_2;
    ynow_2 = ydel_2 + yunit_2;
    Vxunit_2 = [Vxunit_2;-avg_vel*(-sin(2*pi-th(i)))];
    Vyunit_2 = [Vyunit_2;-avg_vel*(cos(2*pi-th(i)))];
    
end

nx  = 1;
nu  = 1;

Q = 1;
R = 1;

Vx = SX.sym('Vx',nu);
Vy = SX.sym('Vy',nu);
Px = SX.sym('Px',nx);
Py = SX.sym('Py',nx);

xdot = Vx(1);
ydot = Vy(1);

f = Function('f', {Px,Vx},{xdot});

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
x_0 = c_2(1)-r; % -4
PX = x_0;
y_0 = c_2(2); % -3.5
PY = y_0;
for i = 1:N
    out_x = RK4(PX,Vx(i));
    PX = out_x;
    
    out_y = RK4(PY,Vy(i));
    PY = out_y;
    
    %J = J + Q*PX(1)^2 + R*Vx(i)^2;
    
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i),Yunit_2(i));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    [xa,ya] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
    ar = polyarea(xa,ya);
    
    wt = (PY - Yunit_2(i))/(Yunit_1(i) - Yunit_2(i));
    
    %steer = atan2(Vy(i),Vx(i));
    
    J = J + Q*((PX(1) - Xunit_2(i))^2  + (PY(1) - Yunit_2(i))^2) + R*wt*ar;  %+ 10*steer;
end

% Terminal constraints: x_0(T)=x_1(T)=0
% Here PX PY are the terminal Robot Positions 
g = [PX;PY;atan2(Vy(2:end),Vx(2:end))-atan2(Vy(1:end-1),Vx(1:end-1))];

% Allocate an NLP solver
nlp = struct('x', [Vx;Vy], 'f', J, 'g', g);

% Create IPOPT solver object
options = struct;
options.hessian_approximation = 'limited-memory';
solver = nlpsol('solver', 'ipopt', nlp);

%% Solve the NLP
arg = struct;
arg.x0  = 0.;    % solution guess
arg.lbx = -0.5;    % lower bound on x
arg.ubx =  0.5;    % upper bound on x
arg.lbg =  0;    % lower bound on g
arg.ubg =  0;    % upper bound on g

% Solve the problem
% Don't keep x0=0, atan2(Vy(1),Vx(1)) will give inf 
res = solver('x0',1,'lbx',[-0.5],'ubx',[0.5],'lbg',[4;-3.5;-0.7*ones(39,1)],'ubg',[4;3.5;0.7*ones(39,1)]);

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
    
    out_y = RK4(y_opt(i-1),u_opt(40 + i-1));
    y_opt(i) = full(out_y);
end

%% Plot results

figure()
subplot(2,1,1)
plot((0:N)*T/N, x_opt)
xlabel('time - t')
ylabel('state - x')
grid on
subplot(2,1,2)
stairs((0:N-1)*T/N, u_opt(1:40),'r')
xlabel('time - t')
ylabel('input - u')
grid on

figure()
subplot(2,1,1)
plot((0:N)*T/N, y_opt)
xlabel('time - t')
ylabel('state - y')
grid on
subplot(2,1,2)
stairs((0:N-1)*T/N, u_opt(41:end),'r')
xlabel('time - t')
ylabel('input - u')
grid on

figure()
plot(x_opt,y_opt)
