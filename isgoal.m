function [goalflag,t,s,bef] = isgoal(newCoor,x_reachest,x_create,Thr,goalPoint)
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
goalflag = 0;
flag1 = 0;
flag2 = 0;
s = 0;
syms x y

eq1 = (x - goalPoint(1))^2 + (y - goalPoint(2))^2 - Thr^2;
if x_create
    k = (newCoor(2) - x_create(2))/(newCoor(1) - x_create(1));
    b = newCoor(2) - k*newCoor(1);
    eq2 = k*x + b - y;
    bef = x_create;
    %k = (newCoor(2) - x_create(2))/(newCoor(1) - x_create(1));
else
    %eq2 = (y - newCoor(2))*(x_reachest(1) - newCoor(1)) - (x_reachest(2) - newCoor(2)*(x - newCoor(1)));
    bef = x_reachest;
    k = (newCoor(2) - x_reachest(2))/(newCoor(1) - x_reachest(1));
    b = newCoor(2) - k*newCoor(1);
    eq2 = k*x + b - y;
end


dis1 = calcuDis(newCoor,bef);
dis2 = calcuDis(newCoor,goalPoint);
dis3 = calcuDis(bef,goalPoint);
dis = abs(k*(goalPoint(1)) - goalPoint(1) + b)/(sqrt(k^2 + 1));
ang1 = (dis1^2 + dis2^2 - dis3^2)/(2*dis1*dis2);
ang2 = (dis1^2 + dis3^2 - dis2^2)/(2*dis1*dis3);

if ang1 > 0 && ang2 > 0 && dis <= Thr
    flag1 = 1;
    s =2;
end

if dis2 < Thr
    flag2 = 1;
    s = 1;
end
    

eq1 = subs(eq1);
eq2 = subs(eq2);
[x,y] = solve(eq1,eq2);
t(1) = double(x(1));
t(2) = double(y(1));

if flag1 || flag2
    goalflag = 1;
end
end

