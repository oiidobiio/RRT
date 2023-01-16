function cubeFlag = isCubeCollision(cubeInfo,nearCoor,newCoor,step)
%% 长方体碰撞检测函数，如果发生碰撞则返回1
 
cubeFlag = 0;
 
if cubeInfo.exist
    for k1 = 1 : size(cubeInfo.axisX,2)
        xMin = cubeInfo.axisX(k1);
        xMax = cubeInfo.axisX(k1)+cubeInfo.length(k1);
        yMin = cubeInfo.axisY(k1);
        yMax = cubeInfo.axisY(k1)+cubeInfo.width(k1);
        zMin = cubeInfo.axisZ(k1);
        zMax = cubeInfo.axisZ(k1)+cubeInfo.height(k1);
        
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
            
            if (xMin<checkPoint(1) && checkPoint(1) < xMax) && (yMin<checkPoint(2) && checkPoint(2) < yMax) && (zMin<checkPoint(3) && checkPoint(3) < zMax)
                cubeFlag = 1;
                return;
            end
        end
        
    end
    
    
end
 
end