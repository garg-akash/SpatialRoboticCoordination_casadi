function lelo = dbg_CheckSide(rect1,rect2,x_rot,y_rot,x_ref,y_ref)

% compute projection of vertex onto perpendicular edge using dot
% product => sign gives direction
side1 = zeros(1,4);
side2 = zeros(1,4);
ind = 0;
    for k=1:4
        side1(k) = (x_rot * (rect1(k,1) - x_ref) + y_rot * (rect1(k,2) - y_ref))
    end
    for k=1:4
        side2(k) = (x_rot * (rect2(k,1) - x_ref) + y_rot * (rect2(k,2) - y_ref))
    end

    % check if vertices are on different sides of the edge
%     if ( (min(side1) >= 0 && max(side2) <= 0) || (max(side1) <= 0 && min(side2) >= 0) || ...
%             (max(side1) < min(side2)) || (max(side2) < min(side1)) )
    if( (max(side1) < min(side2)) || (max(side2) < min(side1)))
        ind = 1;
        lelo = 0;
    else
        if (max(side2) > max(side1))
            ind = 2;
            lelo = max(side1) - min(side2);
        else
            ind = 3;
            lelo = max(side2) - min(side1);
        end
    end
    fprintf('Indicator is %d',ind);
end