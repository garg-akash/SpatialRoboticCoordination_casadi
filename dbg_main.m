%Load reference traj first

addpath('/home/akash/Documents/casadi-linux-matlabR2014a-v3.4.5/')
import casadi.*
Aarr = [];

for i = 1:total_t
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,atan2(Vyunit_2(i),Vxunit_2(i)),Xunit_2(i),Yunit_2(i));
    rect1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    rect2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    p=1;
    w={};
    w0 = [];
    lbw = [];
    ubw = [];
    discrete = [];
    J = 0;
    g={};
    lbg = [];
    ubg = [];
    olp = SX.zeros(1,2*4);
    for j=2:5
        % compute the edge vector
        x = rect1(j,1) - rect1(j-1,1);
        y = rect1(j,2) - rect1(j-1,2);
        %
        x_ref = rect1(j-1,1);
        y_ref = rect1(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        % check on which side the vertices are
        [olp(p), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            dbg_CheckSide_ip(rect1, rect2, x_rot, y_rot,x_ref,y_ref,p);
        disp(olp(p));
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        fprintf('Olp for p=%d ',p)
        p = p + 1;
    end
    % loop through all edges of rect 2 if separation line was not found yet
    
    for j=2:5
        % compute the edge vector
        x = rect2(j,1) - rect2(j-1,1);
        y = rect2(j,2) - rect2(j-1,2);
        %
        x_ref = rect2(j-1,1);
        y_ref = rect2(j-1,2);
        
        % compute the perpendcular to the edge vector
        x_rot = -y;
        y_rot = x;
        
        
        % check on which side the vertices are
        [olp(p), w_r, lbw_r, ubw_r, w0_r, discrete_r, g_r, lbg_r, ubg_r] = ...
            dbg_CheckSide_ip(rect1,rect2, x_rot,y_rot,x_ref,y_ref,p);
        w = {w{:} w_r{:}};
        lbw = [lbw; lbw_r];
        ubw = [ubw; ubw_r];
        w0 = [w0; w0_r];
        discrete = [discrete; discrete_r];
        g = {g{:} g_r{:}};
        lbg = [lbg; lbg_r];
        ubg = [ubg; ubg_r];
        fprintf('Olp for p=%d ',p)
        
        p = p + 1;
    end
    w = vertcat(w{:});
    g = vertcat(g{:});
%      w = w{:};
    lbw = lbw(:);
    ubw = ubw(:);
    w0 = w0(:);
    discrete = discrete(:);
%     g = g{:};
    lbg = lbg(:);
    ubg = ubg(:);
length(w)
length(discrete)
    %J = olp(1)+olp(2)+olp(3)+olp(4)+olp(5)+olp(6)+olp(7)+olp(8); 
    %J = min(olp);
    J = olp(1)*olp(2)*olp(3)*olp(4)*olp(5)*olp(6)*olp(7)*olp(8); 
    nlp_prob = struct('f', J, 'x', w, 'g', g);
    nlp_solver = nlpsol('nlp_solver', 'bonmin', nlp_prob, struct('discrete', discrete));
    
    % Solve the NLP
    sol = nlp_solver('x0',w0, 'lbx',lbw, 'ubx',ubw, 'lbg',lbg, 'ubg',ubg);
    w_opt = full(sol.x);
    fprintf('Min olp for i=%d',i)
    %Aarr = [Aarr;min(olp)];
end
