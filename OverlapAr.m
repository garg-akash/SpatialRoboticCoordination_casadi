%File to calculate overlap area
Ar = [];
Ar_ex = [];
for i = 1:total_t
    [AG_1,BG_1,CG_1,DG_1] = rectangle_plot(l_1,b_1,Hfinal_1(i),Xunit_1(i),Yunit_1(i));
    [AG_2,BG_2,CG_2,DG_2] = rectangle_plot(l_2,b_2,Hfinal_2(i),Xunit_2(i),Yunit_2(i));
    Poly_1 = [AG_1;BG_1;CG_1;DG_1;AG_1];
    Poly_2 = [AG_2;BG_2;CG_2;DG_2;AG_2];
    [xa,ya] = polybool('intersection',Poly_1(:,1),Poly_1(:,2),Poly_2(:,1),Poly_2(:,2));
    arr = polyarea(xa,ya);
    Ar = [Ar,arr];
end
Ar_ex = Ar;
for i=2:total_t
    Ar_ex = [Ar_ex,0];
end