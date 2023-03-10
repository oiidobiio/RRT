%% 清空变量
clear;
clc
tic;
%% 定义变量
axisStart = [0 0];
axisLWH = [200 200];

%定义障碍物
mapInfo.exist = 0;
 
startPoint = [20 20];

goalPoint = [180 180];
 
% mapInfo = createMap(mapInfo);       %创建长方体障碍物信息
mapInfo = createMaze(mapInfo);  
%mapInfo = createTest(mapInfo);
% mapInfo = createMod(mapInfo);

%% 画图
figure(1)
pellucidity = 0.6;    %透明度
hold on;
scatter(startPoint(1),startPoint(2),'MarkerEdgeColor','k','MarkerFaceColor',[1 0 0]);%绘制起点
scatter(goalPoint(1),goalPoint(2),'MarkerEdgeColor','k','MarkerFaceColor','b');%绘制终点
drawmap(mapInfo);
text(startPoint(1),startPoint(2),'Start');%起点文字说明
text(goalPoint(1),goalPoint(2),'End');%终点文字说明
view(2)
grid on;
axis equal;
axis([0 200 0 200 ])
xlabel('x')
ylabel('y')

% mytimer = toc;
 
 
%% 寻找路径
totalPath = [];
%for k1 = 1:size(pathPoint,1)-1

    %Path = RRT(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
   
    Path = RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
   
%     Path = Q_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo);

%     Path = F_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
    
%     Path = FC_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
    
    %Path = M_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
    
%     Path = RRT_C(startPoint,axisStart,axisLWH,goalPoint,mapInfo);
    mytimer = toc;
    disp(mytimer)
    totalfai = 0;
    Path(:,1) = flip(Path(:,1));
    Path(:,2) = flip(Path(:,2));
    for i = 2 : size(Path,1) - 1
        a = [Path(i,1) - Path(i - 1,1) Path(i,2) - Path(i - 1,2)];
        b = [Path(i + 1,1) - Path(i,1) Path(i + 1,2) - Path(i,2)];
        ab = dot(a,b);
        g = a.*a;
        gh = sum(g);
        aa = sqrt(gh);
        g = b.*b;
        gh = sum(g);
        bb = sqrt(gh);
        fai = acos(ab/(aa*bb));
        totalfai = totalfai + fai;
        %Path(1,5) = totalfai;
    end
    disp(totalfai)
    
%end