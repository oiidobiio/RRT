function randCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo)

while 1
    randX = rand*axisLWH(1)+axisStart(1);
    randY = rand*axisLWH(2)+axisStart(2);

    Flag = 0;
    for k1 = 1 : size(mapInfo.axisX,2)
        xMin = mapInfo.axisX(k1) - 0.5;
        xMax = mapInfo.axisX(k1)+mapInfo.width(k1) + 0.5;
        yMin = mapInfo.axisY(k1) - 0.5;
        yMax = mapInfo.axisY(k1)+mapInfo.length(k1) + 0.5;           
        checkPoint = [randX,randY];
            
        if (xMin<checkPoint(1) && checkPoint(1) < xMax) && (yMin<checkPoint(2) && checkPoint(2) < yMax)
            Flag = 1;
        end
    end

if Flag == 0
    randCoor = [randX randY];    
    break
else
    continue
end
end 