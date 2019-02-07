
clc;
clear;
OLA;
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
dist_st = sqrt(power((Xunit_2(1) - Xunit_1(ind(1))),2) + power((Yunit_2(1) - Yunit_1(ind(1))),2));
dist_ed = sqrt(power((Xunit_2(1) - Xunit_1(ind(end))),2) + power((Yunit_2(1) - Yunit_1(ind(end))),2));
if(dist_ed>dist_st)
    ind_or = ind;
else
    ind_or = flipud(ind);
end

N = 9; %planning horizon
nv = 3;
rad_2 = sqrt(power(l_2,2)+power(b_2,2))/2;
rad_1 = sqrt(power(l_1,2)+power(b_1,2))/2;
Nsim = 20;
robo_start = [Xunit_2(1);Yunit_2(1);Hfinal_2(1);0];
robo_traj = [Xunit_2(1);Yunit_2(1)];
robo_vel = 0;
robo_omg = 0;
robo_th = 0;
robo_p = 0;
v_prev = 0;
omg_prev = 0;
ind_ob = 0; %tells index of ind_or which needs to be dealt with
ind_left = 0;
tic;
for k = 1:Nsim
    fprintf('Sim no: %d', k);
    flag1 = zeros(N,1); %tells if we have a critical envelope in current planning horizon
    for i = 1:N
        ind_rb = i;
        for j = 1:length(ind_or)
            dis1 = (sqrt(power((Xunit_2_ex(k+i-1) - Xunit_1(ind_or(j))),2) + power((Yunit_2_ex(k+i-1) - Yunit_1(ind_or(j))),2)) ...
                - rad_1 - rad_2);
            dis2 = sqrt(power((Xunit_2_ex(k+i-1) - Xunit_2(end)),2) + power((Yunit_2_ex(k+i-1) - Yunit_2(end)),2));
            dis3 = sqrt(power((Xunit_2(end) - Xunit_1(ind_or(j))),2) + power((Yunit_2(end) - Yunit_1(ind_or(j))),2));
            if( dis1 < 0.1 && dis3 <= dis2)
                flag1(i:end) = 1;
                ind_ob = j; %TODO for loop corresponding to the critical envelope numbers
                ind_left = length(ind_or) - ind_ob + 1;
                break;
            end
        end
        if(flag1(i)==1)
            break;
        end
    end
%     ind_left = 0;
    [V_opt, Omg_opt, P_opt, Th_opt, Px, Py] = oru_sptl_dev(N, del_t, robo_start, Xunit_1(ind_or), Yunit_1(ind_or), Hfinal_1(ind_or) ...
                                , Xunit_2_ex(k:N+k-1), Yunit_2_ex(k:N+k-1), Hfinal_2_ex(k:N+k-1), l_1, b_1, l_2, b_2 ...
                                , nv, flag1, ind_ob, ind_left, k, v_prev, omg_prev);
    robo_start = [Px(2);Py(2);Th_opt(2);P_opt(2)];
    %     Xunit_2_ex(i+N/nv:N/nv:i+N-1) = Px(3:end);
    %     Yunit_2_ex(i+N/nv:N/nv:i+N-1) = Py(3:end);
%     Hfinal_2_ex(k+N/nv:N/nv:k+N-1) = Th_opt(3:end);
    robo_traj = [robo_traj, [Px(2);Py(2)]];
    robo_vel = [robo_vel, V_opt(1)];
    robo_omg = [robo_omg, Omg_opt(1)];
    robo_p = [robo_p, P_opt(2)];
    robo_th = [robo_th, Th_opt(2)];
    v_prev = V_opt(2:end);
    omg_prev = Omg_opt(2:end);
end
toc;

    