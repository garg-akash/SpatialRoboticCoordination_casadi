
clc;
clear;
warning off;
%OLA;
Dekho;
addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*

if (length(ind) > 0)
    dist_st = sqrt(power((Xunit_2(1) - Xunit_1(ind(1))),2) + power((Yunit_2(1) - Yunit_1(ind(1))),2));
    dist_ed = sqrt(power((Xunit_2(1) - Xunit_1(ind(end))),2) + power((Yunit_2(1) - Yunit_1(ind(end))),2));
    if(dist_ed>dist_st)
        ind_or = ind;
    else
        ind_or = flipud(ind);
    end
else
    ind_or = 1;
end

% N = 9; %planning horizon
% nv = 3;
rad_2 = sqrt(power(l_2,2)+power(b_2,2))/2;
rad_1 = sqrt(power(l_1,2)+power(b_1,2))/2;
robo_start = [Xunit_2(1);Yunit_2(1);Hfinal_2(1);B(1,4)];
robo_traj = [Xunit_2(1);Yunit_2(1)];
robo_vel = vel_0(1);
robo_omg = omg_0(1);
robo_th = B(1,3);
robo_p = B(1,4);
% robo_d = -1*ones(12*N/nv,3);
v_prev = 0;
omg_prev = 0;
ind_ob = 0; %tells index of ind_or which needs to be dealt with
ind_left = 0;
flag2 = 0;
id = 1;
throw = round(N/2,0);
% throw = round(2*N/3,0);
% throw = 0;
% figure
% hax1=axes;
% figure
% hax2=axes;
tic;
% for k = 1:Nsim/2+1
% for k = 1:N:Nsim
for k = 1:N-throw:Nsim
    fprintf('Sim no: %d', k);
%     if (k<=Nsim/2)
%         N = 9;
%     else
%         N = Nsim - k + 1; 
%     end
    flag1 = zeros(N,1); %tells if we have a critical envelope in current planning horizon
    f_b = zeros(N,1);
    for i = 1:N
        ck = (Xunit_2_ex(k+i)-Xunit_2_ex(k+i-1))*cos(Hfinal_2_ex(k+i-1)) + (Yunit_2_ex(k+i)-Yunit_2_ex(k+i-1))*sin(Hfinal_2_ex(k+i-1));
        if(ck>=0)
            f_b(i) = 1;
        end
    end
    for i = 1:N
        %fprintf('At i = %d\n',i);
        ind_rb = i;
        for j = 1:length(ind_or)
            %fprintf('At obs x = %d y = %d\n',Xunit_1(ind_or(j)),Yunit_1(ind_or(j)));
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
    
%     fprintf('Ind_ob: %d', ind_ob);
%     ind_left = 0;
    [V_opt, Omg_opt, P_opt, Th_opt, Px, Py, D_opt] = oru_sptl_dev(N, throw, del_t, robo_start, Xunit_1(ind_or), Yunit_1(ind_or), Hfinal_1(ind_or) ...
                                , Xunit_2_ex(k+1:N+k), Yunit_2_ex(k+1:N+k), Hfinal_2_ex(k+1:N+k), l_1, b_1, l_2, b_2 ...
                                , nv, flag1, ind_ob, ind_left, k, Nsim, cons_A0(k+1:N+k,:), cons_A1(k+1:N+k,:) ...
                                , cons_b(k+1:N+k,:), cons_th(k+1:N+k,:), vel_0(k:N+k-1), omg_0(k:N+k-1) ...
                                , [Xunit_2(end);Yunit_2(end);Hfinal_2(end);B(end,4)], robo_omg(k), robo_vel(k), flag2);
%     robo_start = [Px(2);Py(2);Th_opt(2);P_opt(2)];
% %     Hfinal_2_ex(k+N/nv:N/nv:k+N-1) = Th_opt(3:end);
%     robo_traj = [robo_traj, [Px(2);Py(2)]];
%     robo_vel = [robo_vel, V_opt(1)];
%     robo_omg = [robo_omg, Omg_opt(1)];
%     robo_p = [robo_p, P_opt(2)];
%     robo_th = [robo_th, Th_opt(2)];
%     v_prev = V_opt(2:end);
%     omg_prev = Omg_opt(2:end);
%     flag2 = sqrt(power((Px(2) - Xunit_2(end)),2) ...
%         + power((Py(2) - Yunit_2(end)),2)) - 3;
%     vel_0(k+1:N+k-1) = V_opt(2:end);  %VVI for tracking reference
%     omg_0(k+1:N+k-1) = Omg_opt(2:end);  %VVI for tracking reference
    
%     % To implement OC over each horizon
%     robo_start = [Px(N+1);Py(N+1);Th_opt(N+1);P_opt(N+1)];
%     robo_traj = [horzcat(robo_traj(1,:),Px(2:N+1));horzcat(robo_traj(2,:),Py(2:N+1))];
%     robo_vel = [robo_vel, V_opt(1:N)'];
%     robo_omg = [robo_omg, Omg_opt(1:N)'];
%     robo_p = [robo_p, P_opt(2:N+1)];
%     robo_th = [robo_th, Th_opt(2:N+1)];
%     robo_d(:,id) = D_opt(:); 
%     id = id + 1;
%     flag2 = sqrt(power((Px(N+1) - Xunit_2(end)),2) ...
%         + power((Py(N+1) - Yunit_2(end)),2)) - 3;
    
    robo_start = [Px(N+1-throw);Py(N+1-throw);Th_opt(N+1-throw);P_opt(N+1-throw)];
    robo_traj = [horzcat(robo_traj(1,:),Px(2:N+1-throw));horzcat(robo_traj(2,:),Py(2:N+1-throw))];
    robo_vel = [robo_vel, V_opt(1:N-throw)'];
    robo_omg = [robo_omg, Omg_opt(1:N-throw)'];
    robo_p = [robo_p, P_opt(2:N+1-throw)];
    robo_th = [robo_th, Th_opt(2:N+1-throw)];
%     vel_0(k+N-throw:k+N-1) = V_opt(N+1-throw:end);  %VVI for tracking reference
%     omg_0(k+N-throw:k+N-1) = Omg_opt(N+1-throw:end);  %VVI for tracking reference
    if(k>(Nsim/2-N+throw))
        break;
    end
%     hold on
%     plot(hax1, Px, Py)
%     hold on
%     plot(hax2, robo_traj(1,:), robo_traj(2,:))
 
end
k = k + N-throw;
fprintf('Sim no Exit loop: %d', k);
N = Nsim - k + 1; 
[V_opt, Omg_opt, P_opt, Th_opt, Px, Py, D_opt] = oru_sptl_dev(N, throw, del_t, robo_start, Xunit_1(ind_or), Yunit_1(ind_or), Hfinal_1(ind_or) ...
                                , Xunit_2_ex(k+1:N+k), Yunit_2_ex(k+1:N+k), Hfinal_2_ex(k+1:N+k), l_1, b_1, l_2, b_2 ...
                                , nv, flag1, ind_ob, ind_left, k, Nsim, cons_A0(k+1:N+k,:), cons_A1(k+1:N+k,:) ...
                                , cons_b(k+1:N+k,:), cons_th(k+1:N+k,:), vel_0(k:N+k-1), omg_0(k:N+k-1) ...
                                , [Xunit_2(end);Yunit_2(end);Hfinal_2(end);B(end,4)], robo_omg(k), robo_vel(k), flag2);
robo_traj = [horzcat(robo_traj(1,:),Px(2:N+1));horzcat(robo_traj(2,:),Py(2:N+1))];
robo_vel = [robo_vel, V_opt(1:N)'];
robo_omg = [robo_omg, Omg_opt(1:N)'];
robo_p = [robo_p, P_opt(2:N+1)];
robo_th = [robo_th, Th_opt(2:N+1)];
toc;
th_pp = robo_th;
for i = 1:length(th_pp)
    if (th_pp(i) > pi)
        th_pp(i) = th_pp(i) - 2*pi;
    elseif(th_pp(i) < -pi)
        th_pp(i) = th_pp(i) + 2*pi;
    end
end
% fileID = fopen('path2_opt.txt','w');
% fprintf(fileID,'%f %f %f %f \n',[robo_traj;th_pp;robo_p]);
% fclose(fileID);
   
