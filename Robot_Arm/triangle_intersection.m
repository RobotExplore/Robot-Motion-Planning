% Please see the following links as reference.
% How to decide if there is a point in a triangle:https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
% How to decide if two lines intercept? : https://www.topcoder.com/community/competitive-programming/tutorials/geometry-concepts-line-intersection-and-its-applications/
% Barycentric coordinate is used to see if the point is inside


function flag = triangle_intersection(P1, P2)
%% Triangle_test : returns true if the triangles overlap and false otherwise

for i = 1:3
    for j = 1:3
        A1 = P1(mod(j+1, 3)+1, 2) - P1(mod(j, 3)+1, 2);
        B1 = P1(mod(j, 3)+1, 1) - P1(mod(j+1, 3)+1, 1);
        C1 = A1*P1(mod(j, 3)+1, 1)+B1*P1(mod(j, 3)+1, 2);

        A2 = P2(mod(i+1, 3)+1, 2) - P2(mod(i, 3)+1, 2);
        B2 = P2(mod(i, 3)+1, 1) - P2(mod(i+1, 3)+1, 1);
        C2 = A2*P2(mod(i, 3)+1, 1)+B2*P2(mod(i, 3)+1, 2);

        det = A1*B2 - A2*B1;

        if det == 0
            x = inf;
            y = inf;
        else
            x = (B2*C1 - B1*C2)/det;
            y = (A1*C2 - A2*C1)/det;
        end

        if (min(P1(mod(j, 3)+1, 1), P1(mod(j+1, 3)+1, 1)) <= x) && ....
                (x <= max(P1(mod(j, 3)+1, 1), P1(mod(j+1, 3)+1, 1))) && ....
                (min(P1(mod(j+1, 3)+1, 2), P1(mod(j, 3)+1, 2)) <= y) && ....
                (y <= max(P1(mod(j+1, 3)+1, 2), P1(mod(j, 3)+1, 2)))
            if (min(P2(mod(i, 3)+1, 1), P2(mod(i+1, 3)+1, 1)) <= x) &&....
                    (x <= max(P2(mod(i, 3)+1, 1), P2(mod(i+1, 3)+1, 1))) &&...
                    (min(P2(mod(i+1, 3)+1, 2), P2(mod(i, 3)+1, 2)) <= y) &&...
                    (y <= max(P2(mod(i+1, 3)+1, 2), P2(mod(i, 3)+1, 2)))
                flag1 = true; % The lines intercept
                break;
            end
        else
            flag1 = false;
        end
    end
    if flag1 == true
        break;
    end
end

% For 1 triangle:
flag2 = true;
if flag1 == false
    p1 = [P1(1,1),P1(1,2)];
    p2 = [P1(2,1),P1(2,2)];
    p3 = [P1(3,1),P1(3,2)];
    Flags2 = [];
    
for i = 1:3    
    p = [P2(i,1),P2(i,2)];
    Ifinside=CheckIfInside(p1,p2,p3,p);
    Flags2=[Flags2,Ifinside];
end

    p1 = [P2(1,1),P2(1,2)];
    p2 = [P2(2,1),P2(2,2)];
    p3 = [P2(3,1),P2(3,2)];
    Flags3 = [];
    
for i = 1:3    
    p = [P1(i,1),P1(i,2)];
    Ifinside=CheckIfInside(p1,p2,p3,p);
    Flags3=[Flags3,Ifinside];
end






if (Flags2(1,1)==1)||(Flags2(1,2)==1)||(Flags2(1,3)==1)
    flag2 = true;
else
    flag2 = false;
end

if (Flags3(1,1)==1)||(Flags3(1,2)==1)||(Flags3(1,3)==1)
    flag3 = true;
else
    flag3 = false;
end

end

if (flag1 == 0)&&(flag2 == 0)&&(flag3 == 0)
    flag = false;
else
    flag = true;
end






function Ifinside = CheckIfInside(p1, p2, p3, p)
 x1 = p1(1,1);
 y1 = p1(1,2);
 x2 = p2(1,1);
 y2 = p2(1,2);
 x3 = p3(1,1);
 y3 = p3(1,2);
 xp = p(1,1);
 yp = p(1,2);
 a = ((y2 - y3)*(xp - x3) + (x3 - x2)*(yp - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3));
 b = ((y3 - y1)*(xp - x3) + (x1 - x3)*(yp - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3));
 c = 1 - a - b;
 
 if (a>=0)&&(a<=1)&&(b>=0)&&(b<=1)&&(c>=0)&&(c<=1)
     Ifinside = true;
 else
     Ifinside = false;
 end
    
end

end


