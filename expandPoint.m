function  newCoor = expandPoint(nearCoor,randCoor,step)
 
deltaX = randCoor(1) - nearCoor(1);
deltaY = randCoor(2) - nearCoor(2);
 
fai = atan2(deltaY,deltaX);
 
x = step*cos(fai);
y = step*sin(fai);

newCoor = [x+nearCoor(1) ,y+nearCoor(2)];

end