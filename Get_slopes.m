function [z_x, z_y]=Get_slopes(WORKSPACE, contact_pt, r, z_x_old, z_y_old)
%Returns the partial derivatives (slopes) of the tangent plane at the
%contact_pt
x = contact_pt(1); y = contact_pt(2); z = contact_pt(3);
i = 0; j = 0; k1 = 1; k2 = 1; done1 = false; done2 = false;
while(~done1)
    if(WORKSPACE(x+i,y,z+k1))
        done1 = true;
    elseif(WORKSPACE(x-i,y,z+k1))
        i = -i; done1 = true;
    elseif(WORKSPACE(x+i,y,z-k1))
        k1 = -k1; done1 = true;
    elseif(WORKSPACE(x-i,y,z-k1))
        i = -i; k1 = -k1; done1 = true;
    else
        k1 = k1+1;
        if(k1 > r/5); k1 = 0; i = i+1; end
    end
    if(i > r/5)
        fprintf('WARNING: Distance between points is large\n');
    end
end
z_x = k1/i;

while(~done2)
    if(WORKSPACE(x,y+j,z+k2))
        done2 = true;
    elseif(WORKSPACE(x,y-j,z+k2))
        j = -j; done2 = true;
    elseif(WORKSPACE(x,y+j,z-k2))
        k2 = -k2; done2 = true;
    elseif(WORKSPACE(x,y-j,z-k2))
        j = -j; k2 = -k2; done2 = true;
    else
        k2 = k2+1;
        if(k2 > r/5); k2 = 0; j = j+1; end
    end
    if(k2 > r/5)
        fprintf('WARNING: Distance between points is large\n');
    end
end
z_y = k2/j;
if(i == 0)
    fprintf('WARNING: z_x = Inf, z_x was not updated\n');
    z_x = z_x_old;
end
if(j == 0)
    fprintf('WARNING: z_y = Inf, z_y was not updated\n');
    z_y = z_y_old;
end

end
