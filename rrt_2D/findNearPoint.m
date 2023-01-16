function  [nearCoor,preIndex] = findNearPoint(randCoor,T)
 
tempDis = inf;
calcuDis = @(x,y) sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
 
for k1 = 1:size(T.x,2)
    dis = calcuDis([T.x(k1) T.y(k1)],randCoor);
    if tempDis>dis
        tempDis = dis;
        index = k1;
    end    
 
end
 
nearCoor = [T.x(index) T.y(index)];
preIndex = index;
 
end