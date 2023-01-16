function mapInfo = createMap(mapInfo)
x = 180;
y = 180;
r = 5;
rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','m')
mapInfo.axisX = [50 120];
mapInfo.axisY = [0 70];

mapInfo.length = [130 130];
mapInfo.width = [30 30];

mapInfo.exist = 1;
end

