function mapInfo = createMaze(mapInfo)
x = 180;
y = 180;
r = 5;
rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','m')
mapInfo.axisX = [20 40 40 150 100 140 170 90 100 140 80 140 50 140 90 115 115];
mapInfo.axisY = [150 120 50 20 100 140 120 30 160 170 70 80 70 110 130 65 125];

mapInfo.length = [20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20];
mapInfo.width = [20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20 20];

mapInfo.exist = 1;
end
