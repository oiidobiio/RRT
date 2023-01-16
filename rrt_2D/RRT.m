function Path = RRT(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% RRT算法寻找路径点
 
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 5000;   %最大迭代次数
iter = 0;   %当前迭代次数
step = 2;  %步长
count = 1;  %计数器
Thr = 5;   %阈值
 
%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.pre(1) = 0;
 
while iter < iterMax  %循环迭代
    
    iter = iter+1;
    
    %% 在空间中随机采样
    randCoor = samplePoint(axisStart,axisLWH,goalPoint);
    
    %% 寻找树上最近点
    [nearCoor,preIndex] = findNearPoint(randCoor,T);
    
    %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor,randCoor,step);
    
    %% 碰撞检测 判断是否与长方体、圆柱体、球体发生碰撞
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,step);

    if  contactFlag%若其中一个返回值不为0 则发生碰撞，结束，重新循环
        continue;
    end
    
    %% 将新点插入树中
    count = count+1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.pre(count) = preIndex;
    %将近点与新点连线
    line([nearCoor(1) newCoor(1)],[nearCoor(2) newCoor(2)],'LineWidth',1);  %绘制每一个新点
%     pause(0.01);

    
    if calcuDis(newCoor,goalPoint)<Thr %计算新点与目标点的距离是否达到阈值
        line([newCoor(1) goalPoint(1)],[newCoor(2) goalPoint(2)],'LineWidth',1);
        break;
    end 
    
end
 
if iter==iterMax
    Path = [];
    disp('路径规划失败');
    return;
end
 
%% 寻找路径
index = T.pre(end);
count = 1;
 
while T.pre(index)~=0
    Path(count,1) = T.x(index);
    Path(count,2) = T.y(index);
    index = T.pre(index);
    count = count+1;
end
 
%将初始点添加到Path中
Path(count,1) = startPoint(1);
Path(count,2) = startPoint(2);
 
%将目标点添加到Path中
Path = flipud(Path);
count = count+1;
Path(count,1) = goalPoint(1);
Path(count,2) = goalPoint(2);
 
 
 
end
 
