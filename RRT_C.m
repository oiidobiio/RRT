function Path = RRT_C(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% RRT算法寻找路径点
 
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 1000;   %最大迭代次数
iter = 0;   %当前迭代次数
step = 5;  %步长
count = 1;  %计数器
count_ = 1;
Thr = 5;   %阈值
findpath = 0;
l_draw_newlist = [];
pt_draw_newlist = []; 

%构建树
S.x(1) = startPoint(1);
S.y(1) = startPoint(2);
S.prex(1) = startPoint(1);
S.prey(1) = startPoint(2);
S.pre(1) = 0;

E.x(1) = goalPoint(1);
E.y(1) = goalPoint(2);
E.prex(1) = goalPoint(1);
E.prey(1) = goalPoint(2);
E.pre(1) = 0;

while iter < iterMax
    iter = iter + 1;
    
    %% 在空间中随机采样
    randCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
    
    %% 寻找起点树上的最近点
    [nearCoor,preIndex] = findNearPoint(randCoor,S);

    %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor,randCoor,step);
    
    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,step);
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
        %此处发生逻辑错误，导致无法运行到后面交换语句就重新循环！！！
        if count_ <count
            [S,E,count,count_] = swapp_c(S,E,count,count_);
        end
        continue;
    end
    %% 插入新点到起点树中
    count = count+1;
    S.x(count) = newCoor(1);
    S.y(count) = newCoor(2);
    S.prex(count) = nearCoor(1);
    S.prey(count) = nearCoor(2);
    S.pre(count) = preIndex;
    
    %将近点与新点连线
    line([nearCoor(1) newCoor(1)],[nearCoor(2) newCoor(2)],'Color','b','LineWidth',1);  %绘制每一个新点
%     pause(0.01);
    %% 寻找终点树上的最近点    
    [nearCoor1,preIndex1] = findNearPoint(newCoor,E);
    
    %% 扩展终点树
    newCoor1 = expandPoint(nearCoor1,newCoor,step);
    
    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,nearCoor1,newCoor1,step);
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环
        %此处发生逻辑错误，导致无法运行到后面交换语句就重新循环！！！
        if count_ <count
            [S,E,count,count_] = swapp_c(S,E,count,count_);
        end
        continue;
    end
    
    %% 插入新点到终点树中
    count_ = count_ + 1;
    E.x(count_) = newCoor1(1);
    E.y(count_) = newCoor1(2);
    E.prex(count_) = nearCoor1(1);
    E.prey(count_) = nearCoor1(2);
    E.pre(count_) = preIndex1;
    
    %将近点与新点连线
    line([nearCoor1(1) newCoor1(1)],[nearCoor1(2) newCoor1(2)],'Color','b','LineWidth',1);  %绘制每一个新点
%     pause(0.01);


while calcuDis(newCoor,newCoor1)>Thr
    
    %% 得到终点树新点与起点树新点间的新点
    newCoor2 = expandPoint(newCoor1,newCoor,step);
    contactFlag = iscontact(mapInfo,newCoor1,newCoor2,step);
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环
        break;
    end
    
    %% 插入新点到终点树中
    count_ = count_ + 1;
    E.x(count_) = newCoor2(1);
    E.y(count_) = newCoor2(2);
    E.prex(count_) = newCoor1(1);
    E.prey(count_) = newCoor1(2);
    E.pre(count_) = count_ - 1;
    
    %将近点与新点连线
    line([newCoor1(1) newCoor2(1)],[newCoor1(2) newCoor2(2)],'Color','b','LineWidth',1);  %绘制每一个新点
%     pause(0.01);
    newCoor1 = newCoor2;   
end

% if calcuDis(newCoor1,newCoor)<Thr
%     line([newCoor1(1) newCoor(1)],[newCoor1(2) newCoor(2)],'LineWidth',1);
%     break;
% end

    %% 判断是否到达终点，构成路径
%     size_S = size(S.x,2);
%     size_E = size(E.x,2);
    if calcuDis(newCoor1,newCoor)<Thr
        findpath = 1;
        clear path;
        if S.x(1) == startPoint(1)
            index = count_;
            j = 1;
            while 1
                path.x(j) = E.x(index);
                path.y(j) = E.y(index);
                index = E.pre(index);
                j = j + 1;
                if index == 0
                    break;
                end
            end
            index = count;
            path.x = flip(path.x);
            path.y = flip(path.y);
            while 1
                path.x(j) = S.x(index);
                path.y(j) = S.y(index);
                index = S.pre(index);
                if index == 0
                    break;
                end
                j = j + 1;
            end
            
        else
            index = count;
            j = 1;
            while 1
                path.x(j) = S.x(index);
                path.y(j) = S.y(index);
                index = S.pre(index);
                j = j + 1;
                if index == 0
                    break;
                end
            end
            index = count_;
            path.x = flip(path.x);
            path.y = flip(path.y);
            while 1
                path.x(j) = E.x(index);
                path.y(j) = E.y(index);
                index = E.pre(index);
                if index == 0
                    break;
                end
                j = j + 1;
            end
 
        end
        
    end

    %% 画出路径    
    if findpath
        for j = 2 : size(path.x,2)
            pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end
        totalcost = 0;
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
        break;
    end
end
    
if iter==iterMax
    Path = [];
    disp('路径规划失败');
    return;
end

%% 寻找路径
Path(1,3) = totalcost;
disp(totalcost)
Path(1,4) = count + count_;

for i = 1 : size(path.x,2)
    Path(i,1) = path.x(i);
    Path(i,2) = path.y(i);
end
for j = 2 : size(path.x,2)
    pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
    pt_draw_newlist = [pt_draw_newlist pt_draw_new];
end

% index = size(S.x,2);
% count = 1;
% 
% index_ = size(E.x,2);
% count_ = 1;
% 
% while index ~= 0
%     Path2(count,1) = S.x(index);
%     Path2(count,2) = S.y(index);
%     Path2(count,3) = S.z(index);
%     index = index - 1;
%     count = count + 1;      
% end
% 
% Path2(count,1) = startPoint(1);
% Path2(count,2) = startPoint(2);
% Path2(count,3) = startPoint(3);
% 
% Path2 = flipud(Path2);
% 
% while index_ ~= 0
%     Path2(count_+count,1) = E.x(index_);
%     Path2(count_+count,2) = E.y(index_);
%     Path2(count_+count,3) = E.z(index_);
%     index_ = index_ - 1;
%     count_ = count_+1;      
% end
% 
% Path2(count_+count+1,1) = goalPoint(1);
% Path2(count_+count+1,2) = goalPoint(2);
% Path2(count_+count+1,3) = goalPoint(3);
end