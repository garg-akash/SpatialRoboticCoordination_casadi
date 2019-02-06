addpath('/home/akash/Documents/rawlings-group-octave-mpctools-22c04706f863/')
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*

% Example MPC using 1-norm penalty on x and u.
mpc = import_mpctools();

% Model and sizes.
A = [1 0 ; 0 1];
B = [1 0 ; 0 1];

Nx = size(A, 2);
Nu = size(B, 2);
Nt = 8;
Nsim = 40;

N = struct('x', Nx, 'u', Nu, 't', Nt);

f = mpc.getCasadiFunc(@(x,u) A*x + B*u*del_t, [Nx, Nu], {'x', 'u'}, {'f'});

% Stage cost.

lcasadi = mpc.getCasadiFunc(@l_spatial_ar, {Nx, Nx, Nx, Nx, Nx-1, 1, 1}, ...
                            {'x', 'xsp', 'obsp', 'hsp', 'Q', 'ar', 'AR'}, {'lsp'});
% lcasadi = mpc.getCasadiFunc(@(x, xsp) (x-xsp)'*(x-xsp), [Nx, Nx], ...
%                             {'x', 'xsp'}, {'lsp'});                                               
% Bounds.
lb = struct();
lb.u = -0.5*ones(Nu,1);

ub = struct();
ub.u = 0.5*ones(Nu,1);

%par = struct('xsp', [-3.5;-4]); %Just for the sake of defining unless error at line 34
% Build controller.
% controller = mpc.nmpc('f', f, 'l', lcasadi, 'N', N, 'lb', lb, 'ub', ub, ...
%                      'par', par);

% Simulate.
x = NaN(Nx, Nsim + 1);
x(:,1) = [-4;-3.5];
% x(:,1) = ones(Nx,1);
u = NaN(Nu, Nsim);
ar = 0;
for t = 1:Nsim
    
%     controller.fixvar('x', 1, x(:,t));
%     ar = overlap_ar(t,l_1,b_1,Hfinal_1,Xunit_1,Yunit_1,l_2,b_2,Hfinal_2,Xunit_2,Yunit_2)
    
    par = struct('xsp', [Xunit_2_ex(t:t+Nt-1); Yunit_2_ex(t:t+Nt-1)], 'obsp', ...
        [Xunit_1_ex(t:t+Nt-1); Yunit_1_ex(t:t+Nt-1)], 'hsp', ...
        [Hfinal_1_ex(t:t+Nt-1); Hfinal_2_ex(t:t+Nt-1)], 'Q', 1, 'ar', ar, 'AR', Ar_ex(t:t+Nt-1));
    controller = mpc.nmpc('f', f, 'l', lcasadi, 'N', N, 'lb', lb, 'ub', ub, ...
                     'par', par, 'verbosity', 3);
    %wt = (controller.var.x(2,2) - Yunit_2(t+1))/(Yunit_1(t+1) - Yunit_2(t+1))
    controller.solve();
    fprintf('Step %d: %s\n', t, controller.status);
    
    u(:,t) = controller.var.u(:,1);
    x(:,t+1) = controller.var.x(:,2);
    
    controller.saveguess();
end

