function cost = l_spatial_ar(x, xsp, obsp, hsp, Q, ar, AR)

    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(1.5,0.75,hsp(1),obsp(1),obsp(2));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(1.5,0.75,hsp(2),xsp(1),xsp(2));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
%     [xa,ya] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
%     arr = polyarea(xa,ya);
    
    dx = x - xsp
    wt = (x(2) - xsp(2))/(obsp(2) - xsp(2))
    
    
    cost = 0.3*dx'*Q*dx + wt*AR;
end