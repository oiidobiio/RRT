function Path = RRT_star(startPoint,axisStart,axisLWH,goalPoint,mapInfo)
%% RRT_star算法寻找路径点
 
%% 变量定义
calcuDis = @(x,y)  sqrt((x(1)-y(1))^2+(x(2)-y(2))^2); %计算两点间的距离
iterMax = 2000;   %最大迭代次数
iter = 0;   %当前迭代次数
step = 5;  %步长
count = 1;  %计数器
Thr = 5;   %阈值
RtoNeib = 20;
findpath = 0; 
l_drawlist = [];
pt_drawlist = [];
goal_index = count;
new_totalcost = 1000;

%构建树
T.x(1) = startPoint(1);
T.y(1) = startPoint(2);
T.prex(1) = startPoint(1);
T.prey(1) = startPoint(2);
T.totalcost(1) = 0;
T.pre(1) = 0;

while iter < iterMax
    pause(0.01)
    iter = iter + 1;
    
    %% 在空间中随机采样
    newCoor = samplePoint(axisStart,axisLWH,goalPoint,mapInfo);
    
    %% 寻找树上最近点
    [nearCoor,preIndex] = findNearPoint(newCoor,T);
    
     %% 按照指定步长生成新的扩展点
    %newCoor = expandPoint(nearCoor,randCoor,step);
    
     %% 得到临时父节点的路径代价
    temp_parent = preIndex;
    temp_cost = calcuDis(nearCoor,newCoor) + T.totalcost(temp_parent);
 
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
    
    %% 选择newCoor的父节点，是newCoor路径代价最小
    for cost_index = 1 : length(nearIndexlist)
        cost_New = distoNewlist(cost_index) + T.totalcost(nearIndexlist(cost_index));
        if(cost_New < temp_cost)
            min_cost(1) = T.x(nearIndexlist(cost_index));
            min_cost(2) = T.y(nearIndexlist(cost_index));
            contactFlag = iscontact(mapInfo,min_cost,newCoor,calcuDis(min_cost,newCoor));
            if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                continue;
            end
            temp_cost = cost_New;
            temp_parent = nearIndexlist(cost_index);
        end
    end

    %% 将newCoor插入树中
    count = count + 1;
    T.x(count) = newCoor(1);
    T.y(count) = newCoor(2);
    T.prex(count) = T.x(temp_parent);
    T.prey(count) = T.y(temp_parent);
    T.totalcost(count) = temp_cost;
    T.pre(count) = temp_parent;
    
    l_draw = plot([T.prex(count), newCoor(1)], [T.prey(count), newCoor(2)],'b','Linewidth',0.5);
    %l_draw = plot([T.prex(count),T.prey(count),T.prez(count)],[newCoor(1),newCoor(2),newCoor(3)])
    pt_draw = plot(newCoor(1),newCoor(2),'ko','MarkerSize', 2, 'MarkerFaceColor','k');
    
    l_drawlist = [l_drawlist l_draw];
    pt_drawlist = [pt_drawlist pt_draw]; %生成新点点集
    
    %% 剪枝 重新画线
    if findpath
    for rewire_index = 1 : length(nearIndexlist)
        if(nearIndexlist(rewire_index) ~= temp_parent) %若不是最小路径代价的父节点
            new_cost = temp_cost + distoNewlist(rewire_index);  %计算neib经过newCoor的到起点的路径代价
            if(new_cost < T.totalcost(nearIndexlist(rewire_index))) %判断其与之前树上的路径代价大小，判断是否需要剪枝
                neib(1) = T.x(nearIndexlist(rewire_index));
                neib(2) = T.y(nearIndexlist(rewire_index));
                contactFlag = iscontact(mapInfo,neib,newCoor,calcuDis(neib,newCoor));
                if contactFlag %若其中一个返回值不为0 则发生碰撞，结束，重新循环 
                    continue;
                end 
                T.totalcost(nearIndexlist(rewire_index)) = new_cost;
                T.pre(nearIndexlist(rewire_index)) = count;
                T.prex(nearIndexlist(rewire_index)) = newCoor(1);
                T.prey(nearIndexlist(rewire_index)) = newCoor(2);    
        
                l_drawlist(nearIndexlist(rewire_index)) = plot([T.x(nearIndexlist(rewire_index)), newCoor(1)], [T.y(nearIndexlist(rewire_index)), newCoor(2)],'b','Linewidth',0.5);
                
            end
        end 
    end
    end
    %% 检查是否到达目标点附近
    distoG = calcuDis(newCoor,goalPoint);
    if(distoG < Thr && ~findpath)
        findpath = 1;
        
        count = count + 1;
        goal_index = count;
        T.x(count) = goalPoint(1);
        T.y(count) = goalPoint(2);
        T.prex(count) = newCoor(1);
        T.prey(count) = newCoor(2);
        T.totalcost(count) = T.totalcost(count - 1) + distoG;
        T.pre(count) = count - 1;
        
    end
    
    if(findpath == 1)
        
        path.x(1) = goalPoint(1);
        path.y(1) = goalPoint(2);
        index = T.pre(goal_index);
        j = 2;
        while 2
            path.x(j) = T.x(index);
            path.y(j) = T.y(index);
            index = T.pre(index);
            
            if(index == 0)
                break; 
            end
            j = j + 1;
        end
        for delete_index = 1 : length(pt_drawlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
             delete(pt_drawlist(delete_index));
        end
        
        for j = 2 : size(path.x,2)
            pt_draw = plot([path.x(j);path.x(j-1);],[path.y(j);path.y(j-1);],'g', 'Linewidth', 0.5);
            pt_drawlist = [pt_drawlist pt_draw];
        end
        %break;
        
        totalcost = 0;
        for j = 2 : size(path.x,2)
            cur_dis = sqrt((path.x(j)-path.x(j-1))^2+(path.y(j)-path.y(j-1))^2);
            totalcost = totalcost + cur_dis;
        end
        
        if findpath
            totalfai = 0;
            Q.x = flip(path.x);
            Q.y = flip(path.y);
            for i = 2 : size(Q.x,2) - 1
                a = [Q.x(i) - Q.x(i - 1) Q.y(i) - Q.y(i - 1)];
                b = [Q.x(i + 1) - Q.x(i) Q.y(i + 1) - Q.y(i)];
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
        end        
        
        if new_totalcost > totalcost
            new_totalcost = totalcost;
            new_path = path;
            new_totalfai = totalfai;
            disp("start")
            settimer = toc;
            disp(settimer)
            disp(new_totalcost)
            disp(new_totalfai )
        end

        clear path

    end
    

end

    Path(1,1) = goalPoint(1); 
    Path(1,2) = goalPoint(2); 
    index = T.pre(goal_index);
    Path(1,3) = T.totalcost(goal_index);
    Path(1,4) = goal_index;
    
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
    
    for delete_index = 1 : length(pt_drawlist)%这个地方，对每一个元素进行遍历之后都删除是什么意思？
        delete(pt_drawlist(delete_index));
    end
        
        for j = 2 : size(Path,1)
            pt_draw = plot([Path(j,1);Path(j-1,1);],[Path(j,2);Path(j-1,2);],'g', 'Linewidth', 0.5);
            pt_drawlist = [pt_drawlist pt_draw];
        end
            
end

