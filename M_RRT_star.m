function Path = M_RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% F_RRT_star算法寻找路径点
 
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 5000;   %最大迭代次数
node_count = 5001;
iter = 0;   %当前迭代次数
step = 2;  %步长
count = 1;  %计数器
Thr = 5;   %阈值
RtoNeib = 10;
findpath = 0; 
l_draw_newlist = [];
pt_draw_newlist = [];
l_draw_createlist = [];
pt_draw_createlist = [];
goal_index = count;
D_dich = 2;

%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.prex(1) = startPoint(1);
T.prey(1) = startPoint(2);
T.totalcost(1) = 0;
T.pre(1) = 0;

while iter < iterMax
    iter = iter + 1;

    %% 在空间中随机采样
    randCoor = samplePoint(axisStart,axisLWH,goalPoint);
    
    %% 寻找树上最近点
    [nearCoor,preIndex] = findNearPoint(randCoor,T);
    
     %% 按照指定步长生成新的扩展点
    newCoor = expandPoint(nearCoor,randCoor,step);
    
    %% 得到临时父节点的路径代价
    temp_parent = preIndex;
    temp_cost = step + T.totalcost(temp_parent);
    %% 碰撞检测    
    contactFlag = iscontact(mapInfo,nearCoor,newCoor,calcuDis(nearCoor,newCoor));
    if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环        
        continue;
    end
 
    %% 以newCoor为圆心，以RtoNeib为半径，作圆，搜索节点
    distoNewlist = [];    % 每次循环要把队列清空
    nearIndexlist = [];
    for i = 1:count
        distoNew = sqrt((newCoor(1) - T.x(i))^2 + (newCoor(2) - T.y(i))^2 );
        if(distoNew < RtoNeib)
            distoNewlist = [distoNewlist distoNew];
            nearIndexlist = [nearIndexlist i];
        end
    end
    %% 找到障碍物最近点
    [x_reachest,x_preIndex,p_reachest,p_preIndex,x_cost,p_cost] = findreachest(nearCoor,newCoor,preIndex,T,startPoint,mapInfo,temp_cost);
    
    %% 利用二分法创建靠近障碍物节点
    [x_create,x_cost] = createnode(x_reachest,startPoint,p_reachest,newCoor,mapInfo,D_dich,x_cost,p_cost);
     
    %% 判断是否将创建的节点 x_create 纳入树中
    if x_create
        count = count + 1;
        T.x(count) = newCoor(1);
        T.y(count) = newCoor(2);
        T.prex(count) = x_create(1);
        T.prey(count) = x_create(2);
        T.totalcost(count) = x_cost + calcuDis(newCoor,x_create);
        T.pre(count) = x_preIndex;
        
        T.x(x_preIndex) = x_create(1);
        T.y(x_preIndex) = x_create(2);
        T.prex(x_preIndex) = p_reachest(1);
        T.prey(x_preIndex) = p_reachest(2);
        T.totalcost(x_preIndex) = x_cost;
        T.pre(x_preIndex) = p_preIndex;
        
        l_draw_new = plot([T.prex(count), newCoor(1)], [T.prey(count), newCoor(2)],'b','Linewidth',0.5);
        l_draw_create = plot([T.prex(x_preIndex), x_create(1)], [T.prey(x_preIndex), x_create(2)],'k','Linewidth',0.5);

        pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        pt_draw_create = plot(x_create(1),x_create(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
    
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new]; %生成新点点集   

        
        l_draw_createlist = [l_draw_createlist l_draw_create];
        pt_draw_createlist = [pt_draw_createlist pt_draw_create]; %生成新点点集 
        
%         delete(pt_draw_newlist(x_preIndex));
%         pt_draw_newlist(x_preIndex) = pt_draw_create;
%         
%         delete(l_draw_newlist(x_preIndex));
%         l_draw_newlist(x_preIndex) = l_draw_create;
        
    else
        count = count + 1;
        T.x(count) = newCoor(1);
        T.y(count) = newCoor(2);
        T.prex(count) = x_reachest(1);
        T.prey(count) = x_reachest(2);
        T.totalcost(count) = x_cost;
        T.pre(count) = x_preIndex;
         
        l_draw_new = plot([T.prex(count), newCoor(1)], [T.prey(count), newCoor(2)],'b','Linewidth',0.5);
        pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
        
        l_draw_newlist = [l_draw_newlist l_draw_new];
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end
    
    %% 剪枝 重新划线
    if x_create
    for rewire_index = 1 : length(nearIndexlist)
        if(nearIndexlist(rewire_index) ~= preIndex && nearIndexlist(rewire_index) ~= x_preIndex)
            neib(1) = T.x(nearIndexlist(rewire_index));
            neib(2) = T.y(nearIndexlist(rewire_index));
            temp_cost = T.totalcost(nearIndexlist(rewire_index));
            [x_reachest,x_preIndex,p_reachest,p_preIndex,x_cost,p_cost] = findreachest(newCoor,neib,count,T,startPoint,mapInfo,temp_cost);
            [x_create,x_cost] = createnode(x_reachest,startPoint,p_reachest,neib,mapInfo,D_dich,x_cost,p_cost);    
            if x_create
                T.prex(nearIndexlist(rewire_index)) = x_create(1);
                T.prey(nearIndexlist(rewire_index)) = x_create(2);
                T.totalcost(nearIndexlist(rewire_index)) = x_cost + calcuDis(neib,x_create);
                T.pre(nearIndexlist(rewire_index)) = x_preIndex;
        
                T.x(x_preIndex) = x_create(1);
                T.y(x_preIndex) = x_create(2);
                T.prex(x_preIndex) = p_reachest(1);
                T.prey(x_preIndex) = p_reachest(2);
                T.totalcost(x_preIndex) = x_cost;
                T.pre(x_preIndex) = p_preIndex;
        
                l_draw_new = plot([T.prex(nearIndexlist(rewire_index)), neib(1)], [T.prey(nearIndexlist(rewire_index)), neib(2)],'r','Linewidth',0.5);
                l_draw_create = plot([T.prex(x_preIndex), x_create(1)], [T.prey(x_preIndex), x_create(2)],'r','Linewidth',0.5);

                pt_draw_new = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
                pt_draw_create = plot(x_create(1),x_create(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
    
                l_draw_newlist = [l_draw_newlist l_draw_new];
                pt_draw_newlist = [pt_draw_newlist pt_draw_new]; %生成新点点集   

        
                l_draw_createlist = [l_draw_createlist l_draw_create];
                pt_draw_createlist = [pt_draw_createlist pt_draw_create]; %生成新点点集                 
            else    
                
                T.pre(nearIndexlist(rewire_index)) = x_preIndex;
                T.prex(nearIndexlist(rewire_index)) = x_reachest(1);
                T.prey(nearIndexlist(rewire_index)) = x_reachest(2);
                T.totalcost(count) = x_cost;
        
                l_draw_newlist(nearIndexlist(rewire_index)) = plot([T.x(nearIndexlist(rewire_index)), x_reachest(1)], [T.y(nearIndexlist(rewire_index)),  x_reachest(2)],'r','Linewidth',0.5); 
            end 

        end
    end
    end

    %% 判断是否到达终点，构成路径
    distoG = calcuDis(newCoor,goalPoint);
    if(distoG < Thr && ~findpath)
        findpath = 1;
        
        count = count + 1;
        goal_index = count;
        T.x(count) = goalPoint(1);
        T.y(count) = goalPoint(2);
        T.prex(count) = newCoor(1);
        T.prey(count) = newCoor(2);
        T.pre(count) = count - 1;
        T.totalcost(count) = T.totalcost(count - 1) + distoG;
        
    end
    
    if(findpath == 1)
        path.x(1) = goalPoint(1);
        path.y(1) = goalPoint(2);
        index = T.pre(goal_index);
        j = 2;
        while 1
            path.x(j) = T.x(index);
            path.y(j) = T.y(index);
            index = T.pre(index);
            
            if(index == 0)
                break; 
            end
            j = j + 1;
        end
        for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
             delete(pt_draw_newlist(delete_index));
        end
        
        for j = 2 : size(path.x,2)
            pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_draw_newlist = [pt_draw_newlist pt_draw_new];
        end
        
    end
 end

    Path(1,1) = goalPoint(1); 
    Path(1,2) = goalPoint(2); 
    index = T.pre(goal_index);
    j = 2;    
    while 1        
        Path(j,1) = T.x(index);
        Path(j,2) = T.y(index);
        index = T.pre(index);
            
        if(index == 0)
            break; 
        end
        j = j + 1;
    end
    
    for delete_index = 1 : length(pt_draw_newlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
         delete(pt_draw_newlist(delete_index));
    end
        
    for j = 2 : size(Path,1)
        pt_draw_new = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth',0.5);
        pt_draw_newlist = [pt_draw_newlist pt_draw_new];
    end
end

