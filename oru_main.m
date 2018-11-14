clc;
%clear all;
ReferenceTraj;

addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
if (1)
    Xunit_2 = robo_traj(1,:);
    Yunit_2 = robo_traj(2,:);
    Hfinal_2 = robo_dir;
    Xunit_2_ex(1:40) = Xunit_2;
    Yunit_2_ex(1:40) = Yunit_2;
    Hfinal_2_ex(1:40) = Hfinal_2;
end

OverlapAr;
% Constants
N = length(th)-28; %prediction horizon
T = total_t;
Nsim = 40; %Simulation time length

robo_start = [Xunit_2(1);Yunit_2(1)];
robo_traj = [];
robo_vel = [];
robo_dir = [];
for i = 1:Nsim
    [Vx, Vy, Px, Py] = oru_sptl_OC(N, del_t, robo_start, Xunit_1_ex(i:N+i-1), Yunit_1_ex(i:N+i-1) ...
            , Hfinal_1_ex(i:N+i-1), Xunit_2_ex(i:N+i-1), Yunit_2_ex(i:N+i-1), Hfinal_2_ex(i:N+i-1) ...
            , l_1, b_1, l_2, b_2, Ar_ex(i:N+i-1));
    robo_start = [Px(2);Py(2)];
    robo_traj = [robo_traj, [Px(2);Py(2)]];
    robo_vel = [robo_vel, [Vx(2);Vy(2)]];
    robo_dir = [robo_dir, atan2(Vx(2),Vy(2))];
end
