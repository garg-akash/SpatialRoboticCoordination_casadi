% Inspired by timevaryingmpc.m
%using one shopt nmpc

addpath('/home/akash/Documents/rawlings-group-octave-mpctools-22c04706f863/')
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*

% Example MPC using 1-norm penalty on x and u.
mpc = import_mpctools();

% Reference
c_1 = [0,3.5];
c_2 = [0,-3.5];
r = 4;
total_t = 40;
avg_vel = 2*pi*r/total_t;
th = pi:pi/39:2*pi;
del_t = total_t/(2*length(th));

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
    Xunit_1 = [Xunit_1,xunit_1];
    Yunit_1 = [Yunit_1,yunit_1];
    hfinal_1 = th(i) + hinit_1;
    Hfinal_1 = [Hfinal_1,hfinal_1];
    
    xunit_2 = xnow_2;
    yunit_2 = ynow_2;
    Xunit_2 = [Xunit_2,xunit_2];
    Yunit_2 = [Yunit_2,yunit_2];
    hfinal_2 = th(i) + hinit_2;
    Hfinal_2 = [Hfinal_2,hfinal_2];
    
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

% Model and sizes.
A = [1 0 ; 0 1];
B = [1 0 ; 0 1];

Nx = size(A, 2);
Nu = size(B, 2);
Nt = 10;
Nsim = total_t;

Q = [1];

%N = struct('x', Nx, 'u', Nu, 't', Nt);

f = mpc.getCasadiFunc(@(x,u) A*x + B*u*del_t, [Nx, Nu], {'x', 'u'}, {'f'});

lcasadi = mpc.getCasadiFunc(@l_spatial, {Nx, Nx, Nx-1}, ...
                            {'x', 'xsp', 'Q'}, {'lsp'});

% Build bounds, parameters, and N.
lb = struct('u', -0.5*ones(Nu, Nsim));
ub = struct('u', 0.5*ones(Nu, Nsim));
par = struct('xsp', [Xunit_2; Yunit_2], 'Q', Q);

N = struct('x', Nx, 'u', Nu, 't', Nsim);

x0 = [-4;-3.5]; %Initial Guess

% Build solver and optimize.
solver = mpc.nmpc('f', f, 'l', lcasadi, 'N', N, 'lb', lb, 'ub', ub, ...
                  'x0', x0, 'par', par, 'verbosity', 3);
solver.solve();

mpc.mpcplot('x', solver.var.x, 'u', solver.var.u, ...
            'xsp', solver.par.xsp, 't', solver.var.t);
subplot(1, 2, 1);
legend('x', 'x_{sp}', 'Location', 'SouthEast');
figure
hold on
plot(solver.var.t,solver.var.x);
hold on
plot(Xunit_2);
