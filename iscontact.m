function contactFlag = iscontact(mapInfo,nearCoor,newCoor,step)
   
contactFlag = 0;
 
if mapInfo.exist
    for k1 = 1 : size(mapInfo.axisX,2)
%         xMin = mapInfo.axisX(k1) - 0.5;
%         xMax = mapInfo.axisX(k1)+mapInfo.width(k1) + 0.5;
%         yMin = mapInfo.axisY(k1) - 0.5;
%         yMax = mapInfo.axisY(k1)+mapInfo.length(k1) + 0.5;

        xMin = mapInfo.axisX(k1);
        xMax = mapInfo.axisX(k1)+mapInfo.width(k1);
        yMin = mapInfo.axisY(k1);
        yMax = mapInfo.axisY(k1)+mapInfo.length(k1);

        for k2 = 0:step/2000:step
            deltaX = newCoor(1) - nearCoor(1);
            deltaY = newCoor(2) - nearCoor(2);
 
            fai = atan2(deltaY,deltaX);
 
            x = k2*cos(fai);
            y = k2*sin(fai);
            
            checkPoint = [x+nearCoor(1),y+nearCoor(2)];
            
            if (xMin<checkPoint(1) && checkPoint(1) < xMax) && (yMin<checkPoint(2) && checkPoint(2) < yMax)
                contactFlag = 1;
                return;
            end
        end
    end
    
end

