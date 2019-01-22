clc;
clear all;
Ref_antipodal;
Ref_antipodal_8;
%ReferenceTraj2_2MO_1SO;
%ReferenceTraj2_with_static;
%ReferenceTraj2;
tic;
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
if (0) %To implement multiple optimization
    Xunit_2 = robo_traj(1,:);
    Yunit_2 = robo_traj(2,:);
    Hfinal_2 = robo_dir;
    Xunit_2_ex(1:40) = Xunit_2;
    Yunit_2_ex(1:40) = Yunit_2;
    Hfinal_2_ex(1:40) = Hfinal_2;
end

% Constants
%N = total_t-28; %prediction horizon
N = 12;
nv = 3; %Number of points of interest from all N points
T = total_t;
Nsim = 40 ; %Simulation time length

robo_start = [Xunit_2(1);Yunit_2(1)];
robo_traj = [0;0];
robo_vel = [];
robo_dir = [];
var_all_d1 = [];
var_all_d2 = [];
for i = 1:Nsim
    fprintf('Sim no: %d', i);
    %dGrad tells whether robo is going away from obstacle or coming closer
    if(i<3)
        dGrad = zeros(8,1);
    else
        dGrad(1) = sqrt(power((robo_traj(1,i) - Xunit_1(i-1)),2) + power((robo_traj(2,i) - Yunit_1(i-1)),2)) ...  
                - sqrt(power((robo_traj(1,i-1) - Xunit_1(i-2)),2) + power((robo_traj(2,i-1) - Yunit_1(i-2)),2));
        dGrad(3) = sqrt(power((robo_traj(1,i) - Xunit_3(i-1)),2) + power((robo_traj(2,i) - Yunit_3(i-1)),2)) ...  
                - sqrt(power((robo_traj(1,i-1) - Xunit_3(i-2)),2) + power((robo_traj(2,i-1) - Yunit_3(i-2)),2))
        dGrad(4) = sqrt(power((robo_traj(1,i) - Xunit_4(i-1)),2) + power((robo_traj(2,i) - Yunit_4(i-1)),2)) ...  
                - sqrt(power((robo_traj(1,i-1) - Xunit_4(i-2)),2) + power((robo_traj(2,i-1) - Yunit_4(i-2)),2))
    end
% %     [Vx, Vy, Px, Py, d_values] = WeWillSucceed(N, del_t, robo_start, Xunit_1_ex(i:N+i-1), Yunit_1_ex(i:N+i-1) ...
% %         , Hfinal_1_ex(i:N+i-1), Xunit_2_ex(i:N+i-1), Yunit_2_ex(i:N+i-1), Hfinal_2_ex(i:N+i-1) ...
% %         , l_1, b_1, l_2, b_2, Ar_ex(i:N+i-1));
%     [Vx, Vy, Px, Py, d_values] = oru_sptl_OC_m2_3obs(N, del_t, robo_start, Xunit_1_ex(i:N+i-1), Yunit_1_ex(i:N+i-1) ...
%         , Hfinal_1_ex(i:N+i-1), Xunit_2_ex(i:N+i-1), Yunit_2_ex(i:N+i-1), Hfinal_2_ex(i:N+i-1) ...
%         , Xunit_3_ex(i:N+i-1), Yunit_3_ex(i:N+i-1), Hfinal_3_ex(i:N+i-1), l_1, b_1, l_2, b_2, l_3, b_3);
%     robo_start = [Px(2);Py(2)];
%     Xunit_2_ex(i+1:i+N-1) = Px(3:end);
%     Yunit_2_ex(i+1:i+N-1) = Py(3:end);
%     robo_traj = [robo_traj, [Px(2);Py(2)]];
%     %var_all_d1 = [var_all_d1;var_d1];
%     %var_all_d2 = [var_all_d2;var_d2];
% %     robo_vel = [robo_vel, [Vx(2);Vy(2)]];
% %     robo_dir = [robo_dir, atan2(Vy(2),Vx(2))];
%     robo_vel = [robo_vel, Vx(1)];
%     robo_dir = [robo_dir, Vy(1)];
    [Vx, Vy, Px, Py, d_values] = oru_sptl_OC_m2_3obs_nv(i, N, del_t, robo_start, Xunit_1_ex(i:N+i-1) ...
        , Yunit_1_ex(i:N+i-1), Hfinal_1_ex(i:N+i-1), Xunit_2_ex(i:N+i-1), Yunit_2_ex(i:N+i-1) ...
        , Hfinal_2_ex(i:N+i-1), robo_traj(:,i), Xunit_3_ex(i:N+i-1), Yunit_3_ex(i:N+i-1), Hfinal_3_ex(i:N+i-1) ...
        , Xunit_4_ex(i:N+i-1), Yunit_4_ex(i:N+i-1), Hfinal_4_ex(i:N+i-1), Xunit_5_ex(i:N+i-1), Yunit_5_ex(i:N+i-1) ...
        , Hfinal_5_ex(i:N+i-1), Xunit_7_ex(i:N+i-1), Yunit_7_ex(i:N+i-1), Hfinal_7_ex(i:N+i-1) ...
        ,l_1, b_1, l_2, b_2, l_3, b_3, l_4, b_4, l_5, b_5, l_7, b_7, g_2,nv, dGrad);
    robo_start = [Px(2);Py(2)];
    Xunit_2_ex(i+N/nv:N/nv:i+N-1) = Px(3:end);
    Yunit_2_ex(i+N/nv:N/nv:i+N-1) = Py(3:end);
    Hfinal_2_ex(i+N/nv:N/nv:i+N-1) = Vy(2:end);
    robo_traj = [robo_traj, [Px(2);Py(2)]];
    robo_vel = [robo_vel, Vx(1)];
    robo_dir = [robo_dir, Vy(1)];

end
toc;
figure;
hold on;
plot(robo_traj(1,:),robo_traj(2,:));
% hold on;
% plot(Xunit_2,Yunit_2);
hold on
plot(Xunit_1,Yunit_1);