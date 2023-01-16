function Coor = getCoor(axisStart,axisLWH,goalPoint,mapInfo)

for i = 1 : 100000
    newCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
    Coor.x(i) = newCoor(1);
    Coor.y(i) = newCoor(1);
end

end

