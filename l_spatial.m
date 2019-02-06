function cost = l_spatial(x, xsp, Q)
    dx = x - xsp;
    cost = dx'*Q*dx;
end