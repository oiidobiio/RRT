function sphereFlag = isSphereCollision(sphereInfo,nearCoor,newCoor,step)
 
sphereFlag = 0;
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2+(x(3)-y(3))^2);
 
 
if sphereInfo.exist
   for k1 = 1:size(sphereInfo.centerX,2)
       center = [sphereInfo.centerX(k1) sphereInfo.centerY(k1) sphereInfo.centerZ(k1)];
       
       for k2 = 0:step/400:step
            deltaX = newCoor(1) - nearCoor(1);
            deltaY = newCoor(2) - nearCoor(2);
            deltaZ = newCoor(3) - nearCoor(3);
 
            r = sqrt(deltaX^2+deltaY^2+deltaZ^2);
            fai = atan2(deltaY,deltaX);
            theta = acos(deltaZ/r);
 
            x = k2*sin(theta)*cos(fai);
            y = k2*sin(theta)*sin(fai);
            z = k2*cos(theta);
            
            checkPoint = [x+nearCoor(1),y+nearCoor(2),z+nearCoor(3)];
            if calcuDis(checkPoint,center)<sphereInfo.radius(k1)
                sphereFlag = 1;
                return;
            end
            
        end
       
   end
end
 
end
 