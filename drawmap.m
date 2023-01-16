function drawmap(mapInfo)

for i = 1 : size(mapInfo.axisX,2) 
    
    x1 = mapInfo.axisX(i);
    x2 = mapInfo.axisX(i) + mapInfo.width(i);
    y1 = mapInfo.axisY(i);
    y2 = mapInfo.axisY(i) + mapInfo.length(i); 
   
   fill([x1,x1,x2,x2],...
        [y1,y2,y2,y1],...
        [0,0,0]);
end


% x1 = mapInfo.axisX(1);
% x2 = mapInfo.axisX(1) + mapInfo.width(1);
% 
% x3 = mapInfo.axisX(2);
% x4 = mapInfo.axisX(2) + mapInfo.width(2);
% 
% y1 = mapInfo.axisY(1);
% y2 = mapInfo.axisY(1) + mapInfo.length(1);
% 
% y3 = mapInfo.axisY(2);
% y4 = mapInfo.axisY(2) + mapInfo.length(2);
% 
% fill([x1,x1,x2,x2],...
%      [y1,y2,y2,y1],...
%      [0,0,0]);
%  
% fill([x3,x3,x4,x4],...
%      [y3,y4,y4,y3],...
%      [0,0,0]);
 
end

